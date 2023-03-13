#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include "quadrotor_msgs/PositionCommand.h"

#include <bspline/non_uniform_bspline.h>

#include <active_perception/perception_utils.h>

#include <exploration_manager/faster_types.hpp>

#include <exploration_manager/termcolor.hpp>

#include <mutex>
#include <deque>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::vector;
using std::shared_ptr;
using std::unique_ptr;
using std::string;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, YAWING, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH };

class FastExplorationFSM {
private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_, vis_timer, vis_all_timer_;
  ros::Subscriber trigger_sub_, odom_sub_, pg_T_vio_sub;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  /*  start plan time */
  // double flight_start;
  double flight_finished;

  int adaptive_t = 10;

  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void visualize(const ros::TimerEvent &e);
  void clearVisMarker();

  shared_ptr<PerceptionUtils> percep_utils_;

  


public:
  FastExplorationFSM(/* args */) {
  }
  ~FastExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);

  void cmdCallback();

  void new_thread();

  int k_end_whole;

  bool yawing = false;

  // Loop correction
  Eigen::Matrix3d R_loop;
  Eigen::Vector3d T_loop;
  bool isLoopCorrection = true;

  //flight time
  bool isfinished = false;
  double flight_start;
  double replan_time = 0.0;
  int replan_num = 0;
  double flight_time = 0;
  double flight_distance = 0;
  double flight_CtrCost = 0;
  double cost_start_time = 0;
  double last_start_time = 0;
  bool is_first_replan = true;
  Eigen::Vector3d flight_point;
  Eigen::Vector3d ini_point;

  ros::Time plan_success_time;

  //traj_server

  vector<Eigen::Vector3d> traj_cmd_, traj_real_;
  vector<NonUniformBspline> traj_;
  // yaw control
  double last_yaw_;
  double time_forward_;
  int deltaT;

  int traj_id_;
  bool receive_traj_ = false;
  bool flightEnd = false;

  std::mutex mtx_plan_;

  state new_state;
  state plan_state;

  std::thread t1;

  double plan_cost_time = 0;
  int plan_loop_num = 0;


  std::deque<state> plan_;
  nav_msgs::Odometry odom;
  quadrotor_msgs::PositionCommand cmd;
  // double pos_gain[3] = {5.7, 5.7, 6.2};
  // double vel_gain[3] = {3.4, 3.4, 4.0};
  double pos_gain[3] = {5.7, 5.7, 6.2};
  double vel_gain[3] = {3.4, 3.4, 4.0};

  ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub;

  void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id);
  void drawCmd(const Eigen::Vector3d &pos, const Eigen::Vector3d &vec, const int &id, const Eigen::Vector4d &color);
  void drawFOV(const vector<Eigen::Vector3d> &list1, const vector<Eigen::Vector3d> &list2);
  void visCallback(const ros::TimerEvent &e);

  void pgTVioCallback(geometry_msgs::Pose msg);
    
  bool newPathCallback(vector<state> path, int start_index);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif