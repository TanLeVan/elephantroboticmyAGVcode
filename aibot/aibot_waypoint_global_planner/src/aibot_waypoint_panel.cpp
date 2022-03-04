#include "aibot_waypoint_panel.h"

#include <cstdio>

#include <ros/console.h>

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/qheaderview.h>

namespace aibot_waypoint_panel
{
    AibotWaypointPanel::AibotWaypointPanel(QWidget *parent)
        : rviz::Panel(parent), m_nh(), m_output_goals(1)
    {
        // m_aibot_waypoint_global_planner = m_aibot_waypoint_plugin_loader.createInstance("aibot_waypoint_global_planner::AibotWaypointGlobalPlanner");
        // int temp = m_aibot_waypoint_global_planner->m_goals;

        QVBoxLayout *root_layout = new QVBoxLayout;
        //create a panel about number of goals
        QHBoxLayout *numGoals_layout = new QHBoxLayout;
        numGoals_layout->addWidget(new QLabel("目标数量:"));
        numGoals_layout->addWidget(new QLabel(QString::number(m_output_goals)));
        m_cycle_checkbox = new QCheckBox("循环");
        numGoals_layout->addWidget(m_cycle_checkbox);
        root_layout->addLayout(numGoals_layout);

        //create a table to contain the poseArray
        m_poseArray_table = new QTableWidget;
        initPoseTable();
        root_layout->addWidget(m_poseArray_table);

        //create a manipulate layout
        QHBoxLayout *manipulate_layout = new QHBoxLayout;
        m_output_reset_button = new QPushButton("重置");
        manipulate_layout->addWidget(m_output_reset_button);
        m_output_cancel_button = new QPushButton("取消");
        manipulate_layout->addWidget(m_output_cancel_button);
        m_output_startNavi_button = new QPushButton("开始导航!");
        manipulate_layout->addWidget(m_output_startNavi_button);
        root_layout->addLayout(manipulate_layout);

        setLayout(root_layout);

        // set a Qtimer to start a spin for subscriptions
        QTimer *output_timer = new QTimer(this);
        output_timer->start(200);

        connect(m_output_reset_button, SIGNAL(clicked()), this, SLOT(clearPose()));
        connect(m_output_cancel_button, SIGNAL(clicked()), this, SLOT(cancelNavi()));
        connect(m_output_startNavi_button, SIGNAL(clicked()), this, SLOT(startNavi()));
        connect(m_cycle_checkbox, SIGNAL(clicked(bool)), this, SLOT(checkCycle()));
        connect(output_timer, SIGNAL(timeout()), this, SLOT(startSpin()));

        ros::NodeHandle pnh("aibot_waypoint");
        m_waypoint_sub = pnh.subscribe("/clicked_point", 100, &AibotWaypointPanel::waypointCallback, this);
        m_add_waypoint_pub = pnh.advertise<geometry_msgs::PointStamped>("waypoint_add", 1);
        m_cmd_waypoint_pub = pnh.advertise<std_msgs::UInt8>("waypoint_cmd", 1);
    }

    void AibotWaypointPanel::deleteMark()
    {

    }

    void AibotWaypointPanel::initPoseTable() 
    {
        ROS_INFO("Initialize Aibot Waypoint Pose Table.");
        m_curGoalIdx = 0, m_cycleCnt = 0;
        // permit_ = false, cycle_ = false;
        m_poseArray_table->clear();
        m_pose_array.clear();
        deleteMark();
        m_poseArray_table->setRowCount(m_output_goals);
        m_poseArray_table->setColumnCount(2);
        m_poseArray_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
        m_poseArray_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        QStringList pose_header;
        pose_header << "x" << "y";
        m_poseArray_table->setHorizontalHeaderLabels(pose_header);
        m_cycle_checkbox->setCheckState(Qt::Unchecked);
    }

    void AibotWaypointPanel::updatePoseTable()
    {
        int count = m_pose_array.size();
        QStringList pose_header;
        pose_header << "x" << "y";
        m_poseArray_table->setHorizontalHeaderLabels(pose_header);
        if(count == 0)
        {
            m_poseArray_table->setRowCount(1);
            m_poseArray_table->show();
            return;
        }
        
        m_poseArray_table->setRowCount(count);
        for(int i = 0; i < count; i++)
        {
            geometry_msgs::PoseStamped pose;
            pose = m_pose_array.at(i);
            m_poseArray_table->setItem(i, 0, 
                                        new QTableWidgetItem(QString::number(pose.pose.position.x, 'f', 2)));
            m_poseArray_table->setItem(i, 1, 
                                        new QTableWidgetItem(QString::number(pose.pose.position.y, 'f', 2)));
        }

        m_poseArray_table->show();
    }

    void AibotWaypointPanel::clearPose()
    {
        initPoseTable();
        std_msgs::UInt8 msg;
        msg.data = aibot_waypoint_global_planner::Waypoint_Cmd::Clear;
        m_cmd_waypoint_pub.publish(msg);
    }

    void AibotWaypointPanel::startNavi()
    {
        std_msgs::UInt8 msg;
        msg.data = aibot_waypoint_global_planner::Waypoint_Cmd::Start;
        m_cmd_waypoint_pub.publish(msg);
    }
    
    void AibotWaypointPanel::cancelNavi()
    {
        std_msgs::UInt8 msg;
        msg.data = aibot_waypoint_global_planner::Waypoint_Cmd::Cancel;
        m_cmd_waypoint_pub.publish(msg);
    }

    void AibotWaypointPanel::checkCycle() {
        m_cycle = m_cycle_checkbox->isChecked();
    }

    void AibotWaypointPanel::startSpin() {
        if (ros::ok()) {
            ros::spinOnce();
        }
    }

    void AibotWaypointPanel::waypointCallback(const geometry_msgs::PointStamped::ConstPtr& waypoint)
    {
        ROS_INFO("Aibot Waypoint Panel Add Point.");
        geometry_msgs::PoseStamped pose;
        pose.header = waypoint->header;
        pose.pose.position = waypoint->point;
        pose.pose.orientation.w = 1.0;
        m_pose_array.push_back(pose);
        updatePoseTable();
        m_add_waypoint_pub.publish(waypoint);
    }
}

// 声明此类是一个rviz的插件

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(aibot_waypoint_panel::AibotWaypointPanel, rviz::Panel)