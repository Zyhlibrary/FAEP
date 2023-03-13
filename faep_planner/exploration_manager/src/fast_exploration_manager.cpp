// #include <fstream>
#include <exploration_manager/fast_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <lkh_tsp_solver/lkh_interface.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <active_perception/frontier_finder.h>
#include <plan_manage/planner_manager.h>

#include <exploration_manager/expl_data.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <exploration_manager/termcolor.hpp>

using namespace Eigen;
using namespace termcolor;

namespace fast_planner
{
  // SECTION interfaces for setup and query

  FastExplorationManager::FastExplorationManager()
  {
  }

  FastExplorationManager::~FastExplorationManager()
  {
    ViewNode::astar_.reset();
    ViewNode::caster_.reset();
    ViewNode::map_.reset();
  }

  void FastExplorationManager::initialize(ros::NodeHandle &nh)
  {
    planner_manager_.reset(new FastPlannerManager);
    planner_manager_->initPlanModules(nh);
    edt_environment_ = planner_manager_->edt_environment_;
    sdf_map_ = edt_environment_->sdf_map_;
    frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
    // view_finder_.reset(new ViewFinder(edt_environment_, nh));

    ed_.reset(new ExplorationData);
    ep_.reset(new ExplorationParam);

    nh.param("exploration/refine_local", ep_->refine_local_, true);
    nh.param("exploration/refined_num", ep_->refined_num_, -1);
    nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
    nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
    nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
    nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
    nh.param("exploration/relax_time", ep_->relax_time_, 1.0);

    nh.param("exploration/vm", ViewNode::vm_, -1.0);
    nh.param("exploration/am", ViewNode::am_, -1.0);
    nh.param("exploration/yd", ViewNode::yd_, -1.0);
    nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
    nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

    ViewNode::astar_.reset(new Astar);
    ViewNode::astar_->init(nh, edt_environment_);
    ViewNode::map_ = sdf_map_;

    double resolution_ = sdf_map_->getResolution();
    Eigen::Vector3d origin, size;
    sdf_map_->getRegion(origin, size);
    ViewNode::caster_.reset(new RayCaster);
    ViewNode::caster_->setParams(resolution_, origin);

    planner_manager_->path_finder_->lambda_heu_ = 1.0;
    // planner_manager_->path_finder_->max_search_time_ = 0.05;
    planner_manager_->path_finder_->max_search_time_ = 1.0;

    // Initialize TSP par file
    ofstream par_file(ep_->tsp_dir_ + "/single.par");
    par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
    par_file << "GAIN23 = NO\n";
    par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
    par_file << "RUNS = 1\n";

    // Analysis
    // ofstream fout;
    // fout.open("/home/boboyu/Desktop/RAL_Time/frontier.txt");
    // fout.close();
  }

  void FastExplorationManager::yawingBeforeFlight(const Vector3d &yaw)
  {
    double yaw1;
    if (yaw(0) > 0)
      yaw1 = yaw(0) - 2 * M_PI;
    else
      yaw1 = yaw(0) + 2 * M_PI;
    // Eigen::Vector3d temp_end(yaw1, 0 , 0);
    double circle_t = 1.5 * M_PI * 2 / ViewNode::yd_;
    cout << circle_t << endl;
    planner_manager_->planYawForYawing(yaw, yaw1, true, ep_->relax_time_, circle_t);
  }

  int FastExplorationManager::planExploreMotion(
      const Vector3d &pos, const Vector3d &vel, const Vector3d &acc, const Vector3d &yaw)
  {
    ros::Time t1 = ros::Time::now();
    auto t2 = t1;
    ed_->views_.clear();
    ed_->global_tour_.clear();

    manyViewPoints = false;

    std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
              << ", acc: " << acc.transpose() << std::endl;

    // Search frontiers and group them into clusters
    frontier_finder_->searchFrontiers();

    double frontier_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones' info
    frontier_finder_->computeFrontiersToVisit();
    frontier_finder_->getFrontiers(ed_->frontiers_);
    frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
    frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);

    if (ed_->frontiers_.empty())
    {
      ROS_WARN("No coverable frontier.");
      return NO_FRONTIER;
    }
    frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);

    for (int i = 0; i < ed_->points_.size(); ++i)
      ed_->views_.push_back(
          ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

    double view_time = (ros::Time::now() - t1).toSec();
    ROS_WARN(
        "Frontier: %d, t: %lf, viewpoint: %d, t: %lf", ed_->frontiers_.size(), frontier_time,
        ed_->points_.size(), view_time);

    // Do global and local tour planning and retrieve the next viewpoint
    Vector3d next_pos;
    double next_yaw;
    // Vector3d next_pos_ave;
    vector<double> yawList;
    double middle_yaw;
    if (ed_->points_.size() > 1)
    {
      // Find the global tour passing through all viewpoints
      // Create TSP and solve by LKH
      // Optimal tour is returned as indices of frontier
      vector<int> indices;
      findGlobalTour(pos, vel, yaw, indices);

      vector<double> refined_yaws;
      vector<bool> smallAreaData;

      if (ep_->refine_local_)
      {
        // Do refinement for the next few viewpoints in the global tour
        // Idx of the first K frontier in optimal tour
        t1 = ros::Time::now();

        ed_->refined_ids_.clear();
        ed_->unrefined_points_.clear();
        int knum = min(int(indices.size()), ep_->refined_num_);
        for (int i = 0; i < knum; ++i)
        {
          auto tmp = ed_->points_[indices[i]];
          ed_->unrefined_points_.push_back(tmp);
          ed_->refined_ids_.push_back(indices[i]);
          if ((tmp - pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2)
            break;
        }

        // Get top N viewpoints for the next K frontiers
        ed_->n_points_.clear();
        vector<vector<double>> n_yaws;
        vector<bool> smallArea;
        frontier_finder_->getViewpointsInfo(
            pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws, smallArea);

        ed_->refined_points_.clear();
        ed_->refined_views_.clear();

        refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws, smallAreaData, smallArea);
        next_pos = ed_->refined_points_[0];
        next_yaw = refined_yaws[0];
        // next_pos_ave = refined_Avepos[0];

        // Get marker for view visualization
        for (int i = 0; i < ed_->refined_points_.size(); ++i)
        {
          Vector3d view =
              ed_->refined_points_[i] + 2.0 * Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
          ed_->refined_views_.push_back(view);
        }
        ed_->refined_views1_.clear();
        ed_->refined_views2_.clear();
        for (int i = 0; i < ed_->refined_points_.size(); ++i)
        {
          vector<Vector3d> v1, v2;
          frontier_finder_->percep_utils_->setPose(ed_->refined_points_[i], refined_yaws[i]);
          frontier_finder_->percep_utils_->getFOV(v1, v2);
          ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
          ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
        }
        double local_time = (ros::Time::now() - t1).toSec();
        ROS_WARN("Local refine time: %lf", local_time);
      }
      else
      {
        // Choose the next viewpoint from global tour
        next_pos = ed_->points_[indices[0]];
        next_yaw = ed_->yaws_[indices[0]];
      }

      int frontier_num = 0;
      Eigen::Vector3d real_dir = (ed_->refined_points_[0] - pos).normalized();
      for (int i = 1; i < ed_->refined_points_.size(); i++)
      {
        if ((ed_->refined_points_[i] - pos).norm() < 3.0 && smallAreaData[i] && !planner_manager_->path_finder_->judgePointsCollision(ed_->refined_points_[0], ed_->refined_points_[i]))
        {
          Eigen::Vector3d temp_dir = (ed_->refined_points_[i] - pos).normalized();
          double angle = acos(temp_dir.dot(real_dir)) * 180 / 3.1415926;
          if (angle > 180)
            angle = 360 - angle;
          if (angle < 120)
          {
            frontier_num++;
            yawList.push_back(refined_yaws[i]);
          }
        }
      }
      if (frontier_num >= 1)
        manyViewPoints = true;

    }
    else if (ed_->points_.size() == 1)
    {
      // Only 1 destination, no need to find global tour through TSP
      ed_->global_tour_ = {pos, ed_->points_[0]};
      ed_->refined_tour_.clear();
      ed_->refined_views1_.clear();
      ed_->refined_views2_.clear();

      if (ep_->refine_local_)
      {
        // Find the min cost viewpoint for next frontier
        ed_->refined_ids_ = {0};
        ed_->unrefined_points_ = {ed_->points_[0]};
        ed_->n_points_.clear();
        vector<vector<double>> n_yaws;
        vector<bool> ifSmallArea;
        frontier_finder_->getViewpointsInfo(
            pos, {0}, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws, ifSmallArea);

        double min_cost = 100000;
        int min_cost_id = -1;
        vector<Vector3d> tmp_path;
        for (int i = 0; i < ed_->n_points_[0].size(); ++i)
        {
          auto tmp_cost = ViewNode::computeCost(
              pos, ed_->n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
          if (tmp_cost < min_cost)
          {
            min_cost = tmp_cost;
            min_cost_id = i;
          }
        }
        next_pos = ed_->n_points_[0][min_cost_id];
        next_yaw = n_yaws[0][min_cost_id];
        ed_->refined_points_ = {next_pos};
        ed_->refined_views_ = {next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0)};
        // next_pos_ave = fron_ave[0];
      }
      else
      {
        next_pos = ed_->points_[0];
        next_yaw = ed_->yaws_[0];
        // next_pos_ave = ed_->averages_[0];
      }
    }
    else
      ROS_ERROR("Empty destination.");

    t1 = ros::Time::now();

    // Compute time lower bound of yaw and use in trajectory generation
    double diff = fabs(next_yaw - yaw[0]);
    double time_lb = min(diff, 2 * M_PI - diff) * 1.0 / ViewNode::yd_;
    double time_lb_normal = min(diff, 2 * M_PI - diff) * 1.0 / ViewNode::yd_;
    double time_ratio = -1;
    // double cur_yaw = yaw(0);


    if (manyViewPoints)
    {
      double maxDiff = 0;
      int maxNum = -1;
      for (int i = 0; i < yawList.size(); i++)
      {
        double temp_diff = fabs(yawList[i] - yaw[0]);
        double temp_diff_lb = min(temp_diff, 2 * M_PI - temp_diff);
        if (temp_diff_lb > maxDiff)
        {
          maxDiff = temp_diff_lb;
          maxNum = i;
        }
      }

      middle_yaw = yawList[maxNum];

      double temp_diff2 = fabs(next_yaw - yawList[maxNum]);
      double temp_diff_lb2 = min(temp_diff2, 2 * M_PI - temp_diff2);
      time_lb = (maxDiff + temp_diff_lb2) * 1.3 / ViewNode::yd_;
      time_ratio = maxDiff / (maxDiff + temp_diff_lb2);
    }

    // Generate trajectory of x,y,z
    planner_manager_->path_finder_->reset();
    planner_manager_->path_finder_->cur_vel = vel;
    if (planner_manager_->path_finder_->search(pos, next_pos, true) == Astar::NO_PATH)
    {
      ROS_ERROR("No path to next viewpoint");
      return FAIL;
    }
    ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
    kino_use_shorten = false;
    shortenPath(ed_->path_next_goal_);

    const double radius = 5.0;
    const double len = Astar::pathLength(ed_->path_next_goal_);

    double time_to_firstView = 0.0;
    if (len < 1.5)
    {
      // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based +
      // optimization
      if (manyViewPoints && time_lb < len * 2 / vel.norm())
      {
        planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
      }

      // shortenPath(ed_->path_next_goal_);
      else
      {
        planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb_normal);
        time_lb = time_lb_normal;
        manyViewPoints = false;
      }
      std::cout << "Close goal." << std::endl;
    }
    else
    {
      kino_use_shorten = true;
      shortenPath(ed_->path_next_goal_);
      vector<Eigen::Vector3d> guidePath;
      if (len > radius)
      {
        // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
        // dead end)
        double len = 0.0;
        Vector3d prev_pt = ed_->path_next_goal_.front();

        guidePath.push_back(prev_pt);
        for (int i = 1; i < ed_->path_next_goal_.size() && len < radius; ++i)
        {
          auto cur_pt = ed_->path_next_goal_[i];
          len += (cur_pt - prev_pt).norm();
          prev_pt = cur_pt;
          ed_->next_goal_ = cur_pt;
          guidePath.push_back(cur_pt);
        }
        std::cout << "Far goal." << std::endl;
        len = 10;
      }
      else // Search kino path to exactly next viewpoint and optimize
      {
        ed_->next_goal_ = next_pos;
        guidePath = ed_->path_next_goal_;

        std::cout << "middle goal." << std::endl;
      }

      planner_manager_->guide_path = guidePath;
      planner_manager_->use_guide = true;

      if (manyViewPoints && !(time_lb < len * 2.0 / vel.norm()))
      {
        time_lb = time_lb_normal;
        manyViewPoints = false;
      }

      if (!planner_manager_->kinodynamicReplan(
              pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_to_firstView, time_lb))
        return FAIL;
    }

    // if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1)
    //   ROS_ERROR("Lower bound not satified!");

    cout << bold << green << "Exp time: " << time_lb << ", "
         << "Real time: " << planner_manager_->local_data_.duration_ << reset << endl;

    if (planner_manager_->local_data_.duration_ < time_lb - 0.1 && manyViewPoints)
    {
      ROS_ERROR("Lower bound not satified!");
      cout << bold << red << "Exp time: " << time_lb << ", "
           << "Real time: " << planner_manager_->local_data_.duration_ << reset << endl;
    }

    double path_duration = planner_manager_->local_data_.duration_;
    if (time_ratio == 1 || time_ratio == 0)
      manyViewPoints = false;

    if (manyViewPoints)
    {
      cout << "time_ratio: " << time_ratio << ", " << path_duration << endl;
      double firstTime = time_ratio * path_duration;
      double secTime = path_duration - firstTime;
      first_time_duration = firstTime;
      // Eigen::Vector3d plan_yaw(cur_yaw, yaw(1), yaw(2));
      // planner_manager_->planYawForYawing(plan_yaw, next_yaw, true, ep_->relax_time_, path_duration);

      planner_manager_->planYawExplore(yaw, middle_yaw, true, ep_->relax_time_, firstTime);

      auto info = &planner_manager_->local_data_;
      double mid_yaw = info->yaw_traj_.evaluateDeBoorT(firstTime)[0];
      double mid_dyaw = info->yawdot_traj_.evaluateDeBoorT(firstTime)[0];
      double mid_ddyaw = info->yawdotdot_traj_.evaluateDeBoorT(firstTime)[0];
      if (fabs(mid_ddyaw) > 10)
        mid_ddyaw = yaw[2];
      Eigen::Vector3d middleYaw(mid_yaw, mid_dyaw, mid_ddyaw);

      planner_manager_->planYYawExplore(middleYaw, next_yaw, true, ep_->relax_time_, secTime);
      cout << bold << green << "yaw planning success" << reset << endl;
    }
    else
    {
      planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_, path_duration);
    }

    double traj_plan_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    double yaw_time = (ros::Time::now() - t1).toSec();
    ROS_WARN("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);
    double total = (ros::Time::now() - t2).toSec();
    ROS_WARN("Total time: %lf", total);
    ROS_ERROR_COND(total > 0.1, "Total time too long!!!");

    return SUCCEED;
  }

  void FastExplorationManager::shortenPath(vector<Vector3d> &path)
  {
    if (path.empty())
    {
      ROS_ERROR("Empty path to shorten");
      return;
    }
    // Shorten the tour, only critical intermediate points are reserved.
    const double dist_thresh = 3.0;
    vector<Vector3d> short_tour = {path.front()};
    for (int i = 1; i < path.size() - 1; ++i)
    {
      if (!kino_use_shorten && (path[i] - short_tour.back()).norm() > dist_thresh)
        short_tour.push_back(path[i]);
      else
      {
        // Add waypoints to shorten path only to avoid collision
        ViewNode::caster_->input(short_tour.back(), path[i + 1]);
        Eigen::Vector3i idx;
        while (ViewNode::caster_->nextId(idx) && ros::ok())
        {
          if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
              edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
          {
            short_tour.push_back(path[i]);
            break;
          }
        }
      }
    }
    if ((path.back() - short_tour.back()).norm() > 1e-3)
      short_tour.push_back(path.back());

    // Ensure at least three points in the path
    if (short_tour.size() == 2)
      short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
    path = short_tour;
  }

  void FastExplorationManager::findGlobalTour(
      const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d cur_yaw,
      vector<int> &indices)
  {
    auto t1 = ros::Time::now();

    // Get cost matrix for current state and clusters
    Eigen::MatrixXd cost_mat;
    frontier_finder_->updateFrontierCostMatrix();
    frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
    const int dimension = cost_mat.rows();

    double mat_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // Write params and cost matrix to problem file
    ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
    // Problem specification part, follow the format of TSPLIB

    string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
                       "\nEDGE_WEIGHT_TYPE : "
                       "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

    // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " + to_string(dimension) +
    //     "\nEDGE_WEIGHT_TYPE : "
    //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

    prob_file << prob_spec;
    // prob_file << "TYPE : TSP\n";
    // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
    // Problem data part
    const int scale = 100;
    if (false)
    {
      // Use symmetric TSP
      for (int i = 1; i < dimension; ++i)
      {
        for (int j = 0; j < i; ++j)
        {
          int int_cost = cost_mat(i, j) * scale;
          prob_file << int_cost << " ";
        }
        prob_file << "\n";
      }
    }
    else
    {
      // Use Asymmetric TSP
      for (int i = 0; i < dimension; ++i)
      {
        for (int j = 0; j < dimension; ++j)
        {
          int int_cost = cost_mat(i, j) * scale;
          prob_file << int_cost << " ";
        }
        prob_file << "\n";
      }
    }

    prob_file << "EOF";
    prob_file.close();

    // Call LKH TSP solver
    solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

    // Read optimal tour from the tour section of result file
    ifstream res_file(ep_->tsp_dir_ + "/single.txt");
    string res;
    while (getline(res_file, res))
    {
      // Go to tour section
      if (res.compare("TOUR_SECTION") == 0)
        break;
    }

    if (false)
    {
      // Read path for Symmetric TSP formulation
      getline(res_file, res); // Skip current pose
      getline(res_file, res);
      int id = stoi(res);
      bool rev = (id == dimension); // The next node is virutal depot?

      while (id != -1)
      {
        indices.push_back(id - 2);
        getline(res_file, res);
        id = stoi(res);
      }
      if (rev)
        reverse(indices.begin(), indices.end());
      indices.pop_back(); // Remove the depot
    }
    else
    {
      // Read path for ATSP formulation
      while (getline(res_file, res))
      {
        // Read indices of frontiers in optimal tour
        int id = stoi(res);
        if (id == 1) // Ignore the current state
          continue;
        if (id == -1)
          break;
        indices.push_back(id - 2); // Idx of solver-2 == Idx of frontier
      }
    }

    res_file.close();

    // Get the path of optimal tour from path matrix
    frontier_finder_->getPathForTour(cur_pos, indices, ed_->global_tour_);

    double tsp_time = (ros::Time::now() - t1).toSec();
    ROS_WARN("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
  }

  void FastExplorationManager::refineLocalTour(
      const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
      const vector<vector<Vector3d>> &n_points, const vector<vector<double>> &n_yaws,
      vector<Vector3d> &refined_pts, vector<double> &refined_yaws, vector<bool> &smallAreaData, vector<bool> &smallArea)
  {
    double create_time, search_time, parse_time;
    auto t1 = ros::Time::now();

    // Create graph for viewpoints selection
    GraphSearch<ViewNode> g_search;
    vector<ViewNode::Ptr> last_group, cur_group;

    // Add the current state
    ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0], false));
    first->vel_ = cur_vel;
    g_search.addNode(first);
    last_group.push_back(first);
    ViewNode::Ptr final_node;
    int node_num = 0;

    // Add viewpoints
    std::cout << "Local tour graph: ";
    for (int i = 0; i < n_points.size(); ++i)
    {
      // //仅优化距离当前位置小于5m的视点
      // if ((cur_pos - n_points[i].front()).norm() > 5)
      // {
      //   //保证有下一个目标点
      //   if ((i == n_points.size() - 1) && node_num == 0)
      //   {
      //     ViewNode::Ptr node(new ViewNode(n_points[0][0], n_yaws[0][0], avergae_pos[0]));
      //     g_search.addNode(node);
      //     // Connect a node to nodes in last group
      //     for (auto nd : last_group)
      //       g_search.addEdge(nd->id_, node->id_);

      //     final_node = node;
      //   }
      //   continue;
      // }

      // node_num++;

      // Create nodes for viewpoints of one frontier
      for (int j = 0; j < n_points[i].size(); ++j)
      {
        ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j], smallArea[i]));
        g_search.addNode(node);
        // Connect a node to nodes in last group
        for (auto nd : last_group)
          g_search.addEdge(nd->id_, node->id_);
        cur_group.push_back(node);

        // Only keep the first viewpoint of the last local frontier
        if (i == n_points.size() - 1)
        {
          final_node = node;
          break;
        }
      }
      // Store nodes for this group for connecting edges
      std::cout << cur_group.size() << ", ";
      last_group = cur_group;
      cur_group.clear();
    }
    std::cout << "" << std::endl;
    create_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // Search optimal sequence
    vector<ViewNode::Ptr> path;
    g_search.DijkstraSearch(first->id_, final_node->id_, path);

    search_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // Return searched sequence
    for (int i = 1; i < path.size(); ++i)
    {
      refined_pts.push_back(path[i]->pos_);
      refined_yaws.push_back(path[i]->yaw_);
      smallAreaData.push_back(path[i]->smallArea);
    }

    // Extract optimal local tour (for visualization)
    ed_->refined_tour_.clear();
    ed_->refined_tour_.push_back(cur_pos);
    ViewNode::astar_->lambda_heu_ = 1.0;
    ViewNode::astar_->setResolution(0.2);
    for (auto pt : refined_pts)
    {
      vector<Vector3d> path;
      if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
        ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
      else
        ed_->refined_tour_.push_back(pt);
    }
    ViewNode::astar_->lambda_heu_ = 10000;

    parse_time = (ros::Time::now() - t1).toSec();
    // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
  }

} // namespace fast_planner
