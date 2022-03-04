#ifndef AIBOT_WAYPOINT_GLOBAL_PLANNER_H
#define AIBOT_WAYPOINT_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalID.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

namespace aibot_waypoint_global_planner
{
    typedef enum Waypoint_Cmd
    {
        Start = 0,
        Cancel = 1,
        Clear = 2,
        Pause = 3
    }WaypointCmd;

    class AibotWaypointGlobalPlanner: public nav_core::BaseGlobalPlanner
    {
        public: 
            
            AibotWaypointGlobalPlanner();

            AibotWaypointGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

            ~AibotWaypointGlobalPlanner();

            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

            bool makePlan(const geometry_msgs::PoseStamped& start_pose, 
            const geometry_msgs::PoseStamped& goal, 
            std::vector<geometry_msgs::PoseStamped>& plan);

            void waypointCallback(const geometry_msgs::PointStamped::ConstPtr& waypoint);
            
            void cmdWaypointCallback(const std_msgs::UInt8::ConstPtr& msg);

            void externalPathCallback(const nav_msgs::PathConstPtr& plan);

            void createAndPublishMarkersFromPath(const std::vector<geometry_msgs::PoseStamped>& path);

            void interpolatePath(nav_msgs::Path& path);

            std::vector<geometry_msgs::PoseStamped> getWaypoint();
        
        private:

            bool initialized_;
            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D* costmap_;
            base_local_planner::WorldModel* world_model_;

            // subscribers and publishers
            ros::Subscriber waypoint_add_sub_;  // subscriber of manually inserted waypoints
            ros::Subscriber waypoint_cmd_sub_;
            ros::Subscriber external_path_sub_;  // subscriber of external input path
            ros::Publisher waypoint_marker_pub_;  // publisher of waypoint visualization markers
            ros::Publisher goal_pub_;  // publisher of goal corresponding to the final waypoint
            ros::Publisher cancel_goal_;  //publisher of cancel goal
            ros::Publisher plan_pub_;  // publisher of the global plan

            // configuration parameters
            double epsilon_;  // distance threshold between two waypoints that signifies the last waypoint
            int waypoints_per_meter_;  // number of waypoints per meter of generated path used for interpolation

            // containers
            std::vector<geometry_msgs::PoseStamped> waypoints_;  //!< container for the manually inserted waypoints
            nav_msgs::Path path_;  // container for the generated interpolated path

            //flags
            bool clear_waypoints_;  // flag indicating that the waypoint container must be cleared to start anew
    };
} // namespace aibot_waypoint_global_planner



#endif  // AIBOT_WAYPOINT_GLOBAL_PLANNER_H