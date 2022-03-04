#ifndef AIBOT_WAYPOINT_PANEL_H
#define AIBOT_WAYPOINT_PANEL_H

#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <rviz/panel.h>

#include <QPushButton>
#include <QTableWidget>
#include <QCheckBox>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_datatypes.h>

#include <aibot_waypoint_global_planner/aibot_waypoint_global_planner.h>

namespace aibot_waypoint_panel
{
    
    class AibotWaypointPanel: public rviz::Panel
    {
        Q_OBJECT

        public:
            explicit AibotWaypointPanel(QWidget *parent = 0);
        
        public Q_SLOTS:

            void deleteMark();

        protected Q_SLOTS:

            void updateMaxNumGoal();             // update max number of goal
            void initPoseTable();               // initialize the pose table
            void updatePoseTable();             // update the pose table
            void clearPose();
            void startNavi();                   // start navigate for the first pose
            void cancelNavi();

            void checkCycle();
            void completeNavi();               //after the first pose, continue to navigate the rest of poses
            void cycleNavi();

            bool checkGoal(std::vector<actionlib_msgs::GoalStatus> status_list);  // check whether arrived the goal

            static void startSpin(); // spin for sub

            void waypointCallback(const geometry_msgs::PointStamped::ConstPtr& waypoint);

        protected:

            QCheckBox *m_cycle_checkbox;
            QTableWidget *m_poseArray_table;
            QPushButton *m_output_reset_button, *m_output_startNavi_button, *m_output_cancel_button;

            ros::NodeHandle m_nh;

            int m_output_goals;
            int m_curGoalIdx = 0, m_cycleCnt = 0;
            bool m_cycle = false;

            std::vector<geometry_msgs::PoseStamped> m_pose_array;

            ros::Subscriber m_waypoint_sub;  // subscriber of manually inserted waypoints
            ros::Publisher m_add_waypoint_pub;      //publisher for add waypoints
            ros::Publisher m_cmd_waypoint_pub;      //publihser for sending waypoints commands
    };
}

#endif 