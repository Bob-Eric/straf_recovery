#include "straf_recovery/straf_recovery.h"
#include "obstacle_finder/obstacle_finder.h"
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
namespace straf_recovery {
  StrafRecovery::StrafRecovery() : initialized_(false), cycles_(0) {}

  void StrafRecovery::initialize(std::string name, tf2_ros::Buffer* tf2_buffer,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap) {
    if (initialized_) {
      ROS_ERROR("thou shall not call initialize twice on this object. Doing "
        "nothing.");
      return;
    }

    initialized_ = true;
    find_obstacle_time_old_ = 0;
    name_ = name;
    tfbuffer_ = tf2_buffer;
    local_costmap_ = local_costmap;
    global_costmap_ = global_costmap;
    costmap_ = local_costmap->getCostmap();
    local_costmap_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    // get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);
    ros::NodeHandle nh;
    ros::NodeHandle base_local_planner_nh("~/TrajectoryPlannerROS");

    // setup dynamic reconfigure
    dsrv_ = new dynamic_reconfigure::Server<StrafRecoveryConfig>(private_nh);
    dynamic_reconfigure::Server<StrafRecoveryConfig>::CallbackType cb =
      boost::bind(&StrafRecovery::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    private_nh.param("enabled", enabled_, true);
    private_nh.param("go_to_goal_distance_threshold", go_to_goal_distance_threshold_, 0.0);
    private_nh.param("minimum_translate_distance", minimum_translate_distance_, 0.5);
    private_nh.param("maximum_translate_distance", maximum_translate_distance_, 5.0);
    private_nh.param("straf_vel", vel_, 0.1);
    private_nh.param("timeout", timeout_, 10);

    // use the same control frequency and xy goal tolerance as the base local planner
    base_local_planner_nh.param("frequency", frequency_, 20.0);
    base_local_planner_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.1);

    // for visualizing
    obstacle_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("obstacle_direction", 10);

    // for failure detection
    cycles_pub_ = private_nh.advertise<std_msgs::Int32>("cycles", 10);

    ROS_INFO("minimum_translate_distance %f", minimum_translate_distance_);
    ROS_INFO("maximum_translate_distance %f", maximum_translate_distance_);
    ROS_INFO("vel %f", vel_);

    // use the same control frequency as the base local planner
    base_local_planner_nh.param("frequency", frequency_, 20.0);

    // knowing current goal means we can cheat and straf to the goal
    goal_sub_ = nh.subscribe("move_base_simple/goal", 10, &StrafRecovery::goalCallback, this);
  }

  void StrafRecovery::runBehavior() {
    if (!initialized_) {
      ROS_ERROR("thou shall not fail to call initialize! Doing nothing.");
      return;
    }

    if (!enabled_) {
      ROS_INFO("skipping %s because it's disabled.", name_.c_str());
      return;
    }

    if (local_costmap_ == NULL) {
      ROS_ERROR("Thou shall not pass costmaps to the straf recovery which are nullptr. "
        "Doing nothing");
      return;
    }

    ROS_WARN("Entering straf recovery behavior.");

    ros::NodeHandle n;
    ros::Rate r(frequency_);
    vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    geometry_msgs::PoseStamped initial_local_pose_msg;
    local_costmap_->getRobotPose(initial_local_pose_msg);

    double current_distance_translated = 0.0;

    tf2::Vector3 initial_local_pose(
      initial_local_pose_msg.pose.position.x,
      initial_local_pose_msg.pose.position.y,
      initial_local_pose_msg.pose.position.z);


    obstacle_finder::ObstacleFinder finder(local_costmap_);

    ros::Time end = ros::Time::now() + ros::Duration(timeout_);
    unsigned int local_pose_cost;
    obstacle_finder::Obstacle nearest_obstacle;
    double desired_translate_distance = minimum_translate_distance_;

    while (n.ok() && ros::Time::now() < end) {

      geometry_msgs::PoseStamped local_pose_msg;
      local_costmap_->getRobotPose(local_pose_msg);

      tf2::Vector3 local_pose(
        local_pose_msg.pose.position.x,
        local_pose_msg.pose.position.y,
        local_pose_msg.pose.position.z);

      double robot_odom_x = local_pose.getX();
      double robot_odom_y = local_pose.getY();

      current_distance_translated = (local_pose - initial_local_pose).length();

      tf2::Vector3 last_goal_pose(
        last_goal_.pose.position.x,
        last_goal_.pose.position.y,
        last_goal_.pose.position.z);


      double distance_to_goal = (last_goal_pose - local_pose).length();

      ROS_DEBUG("distance_to_goal: %f", distance_to_goal);

      unsigned int robot_map_x, robot_map_y;
      costmap_->worldToMap(robot_odom_x, robot_odom_y, robot_map_x, robot_map_y);
      local_pose_cost = costmap_->getCost(robot_map_x, robot_map_y);
      // ROS_INFO("local_pose_cost: %d", local_pose_cost);

      // #######################   Unimplemented   #######################
      if (distance_to_goal < go_to_goal_distance_threshold_) {
        ROS_INFO("close enough to goal. strafing there instead");
        tf2::Vector3 goal_pose(last_goal_.pose.position.x, last_goal_.pose.position.y, last_goal_.pose.position.z);
        strafInDiretionOfPose(local_pose_msg, goal_pose, false); //straf towards not away
        if (distance_to_goal < xy_goal_tolerance_) {
          return;
        }
      }
      // #######################   Unimplemented   #######################
      else {

        // check if we've reached max distance
        if (current_distance_translated > maximum_translate_distance_) {
          ROS_WARN("Straf Recovery has met maximum translate distance");
          return;
        }


        if (current_distance_translated > desired_translate_distance) {
          // check if robot pose is in free space
          if (local_pose_cost == costmap_2d::FREE_SPACE) {
            ROS_WARN("Straf Recovery has met desired translate distance %.2f and robot is in free space, exiting",
              desired_translate_distance);
            return;
          }
          else {
            const double increase_distance_threshold = 0.1;
            ROS_WARN("Straf Recovery has met desired translate distance %.2f but robot is not in free space, increasing desired distance to %.2f",
              desired_translate_distance, desired_translate_distance + increase_distance_threshold);
            desired_translate_distance += increase_distance_threshold;
          }
        }
        // if robot is inside the obstacle, go to the nearest obstacle;
        // otherwise, go away from the nearest obstacle
        bool inside_obstacle = local_pose_cost > costmap_2d::LETHAL_OBSTACLE;
        if (inside_obstacle) {
          ROS_ERROR("Robot is inside the obstacle, going to the nearest obstacle");
        }
        else {
          ROS_INFO("Robot is outside the obstacle, going away from the nearest obstacle");
        }

        // The obstacle finder has a pointer to the local costmap, so that will keep working.
        // We just need to use the opdated x and y
        const double find_obstacle_time_threshold = 2.0;
        const bool find_boundary = inside_obstacle;
        if (ros::Time::now().toSec() - find_obstacle_time_old_ > find_obstacle_time_threshold) {
          find_obstacle_time_old_ = ros::Time::now().toSec();
          nearest_obstacle = finder.nearestObstacle(robot_odom_x, robot_odom_y, find_boundary);
        }
        // nearest_obstacle = finder.nearestObstacle(robot_odom_x, robot_odom_y, find_boundary);

        tf2::Vector3 obstacle_pose(nearest_obstacle.x, nearest_obstacle.y, local_pose.getZ());
        const bool away_from_obstacle = !inside_obstacle;
        strafInDiretionOfPose(local_pose_msg, obstacle_pose, away_from_obstacle);
      }

      r.sleep();
    }

    cycles_++;
    std_msgs::Int32 msg;
    msg.data = cycles_;
    cycles_pub_.publish(msg);
  }

  void StrafRecovery::strafInDiretionOfPose(geometry_msgs::PoseStamped current_pose, tf2::Vector3 obstacle_pose, bool away) {
    tf2::Vector3 current_pose_vector(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

    tf2::Vector3 diff = (current_pose_vector - obstacle_pose) * (away ? 1 : -1);
    double yaw_in_odom_frame = atan2(diff.y(), diff.x());

    tf::Quaternion straf_direction = tf::createQuaternionFromYaw(yaw_in_odom_frame);

    geometry_msgs::PoseStamped obstacle_msg;

    obstacle_msg.header.frame_id = current_pose.header.frame_id;
    obstacle_msg.header.stamp = ros::Time(0);
    obstacle_msg.pose.position.x = obstacle_pose.getX();
    obstacle_msg.pose.position.y = obstacle_pose.getY();
    obstacle_msg.pose.orientation.x = straf_direction.x();
    obstacle_msg.pose.orientation.y = straf_direction.y();
    obstacle_msg.pose.orientation.z = straf_direction.z();
    obstacle_msg.pose.orientation.w = straf_direction.w();

    obstacle_pub_.publish(obstacle_msg);

    geometry_msgs::PoseStamped straf_msg;

    tfbuffer_->transform(obstacle_msg, straf_msg, "base_link");

    // angle in the base_link frame
    double straf_angle = tf::getYaw(straf_msg.pose.orientation);



    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vel_ * cos(straf_angle);
    cmd_vel.linear.y = vel_ * sin(straf_angle);
    cmd_vel.linear.z = 0.0;

    vel_pub_.publish(cmd_vel);
  }

  void StrafRecovery::goalCallback(const geometry_msgs::PoseStamped& msg) {
    ROS_INFO_ONCE("Received goal");
    last_goal_ = msg;
  }

  void StrafRecovery::reconfigureCB(StrafRecoveryConfig& config, uint32_t level) {
    enabled_ = config.enabled;
    timeout_ = config.timeout;
    minimum_translate_distance_ = config.minimum_translate_distance;
    maximum_translate_distance_ = config.maximum_translate_distance;
    xy_goal_tolerance_ = config.xy_goal_tolerance;
    go_to_goal_distance_threshold_ = config.go_to_goal_distance_threshold;
    vel_ = config.straf_vel;
    frequency_ = config.frequency;
  }
  StrafRecovery::~StrafRecovery() {
    delete dsrv_;
    delete local_costmap_model_;
  }

}

PLUGINLIB_EXPORT_CLASS(straf_recovery::StrafRecovery, nav_core::RecoveryBehavior)
