#include <iostream>

// ROS
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PatrolNode
{
public:
    // ros::Publisher pub_mb_goal;
    PatrolNode(ros::NodeHandle nh);
    void Target_one();
    void Target_two();
    void Target_home();
    void Setting_patrol_path();
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

    ros::Subscriber odom_sub;
};

PatrolNode::PatrolNode(ros::NodeHandle nh)
{
    odom_sub = nh.subscribe("/odom_combined", 1, &PatrolNode::odom_callback, this);
    Setting_patrol_path();
}

void PatrolNode::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // cout<<"Position"<<endl;
    // cout<<"X: "<<msg->pose.pose.position.x<<endl;
    // cout<<"Y: "<<msg->pose.pose.position.y<<endl;
    // cout<<"Z: "<<msg->pose.pose.position.z<<endl;
    // cout<<"Orientation"<<endl;
    // cout<<"rx: "<<msg->pose.pose.orientation.x<<endl;
    // cout<<"ry: "<<msg->pose.pose.orientation.y<<endl;
    // cout<<"rz: "<<msg->pose.pose.orientation.z<<endl;
}

void PatrolNode::Target_one()
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 3.228;
    goal.target_pose.pose.position.y = -4.406;
    goal.target_pose.pose.position.z = 0.000;
    goal.target_pose.pose.orientation.x = 0.000;
    goal.target_pose.pose.orientation.y = 0.000;
    goal.target_pose.pose.orientation.z = -0.484;
    goal.target_pose.pose.orientation.w = 0.875;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the goal");
    else
        ROS_INFO("The base failed to move to the goal");
}

void PatrolNode::Target_two()
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 3.106;
    goal.target_pose.pose.position.y = 1.744;
    goal.target_pose.pose.position.z = 0.000;
    goal.target_pose.pose.orientation.x = 0.000;
    goal.target_pose.pose.orientation.y = 0.000;
    goal.target_pose.pose.orientation.z = 0.971;
    goal.target_pose.pose.orientation.w = -0.238;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the goal");
    else
        ROS_INFO("The base failed to move to the goal");
}

void PatrolNode::Target_home()
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.353;
    goal.target_pose.pose.position.y = 1.130;
    goal.target_pose.pose.position.z = 0.000;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.226;
    goal.target_pose.pose.orientation.w = 0.974;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the goal");
    else
        ROS_INFO("The base failed to move to the goal");
}

void PatrolNode::Setting_patrol_path()
{
    while(true)
    {
        char c = getchar();

        if(c == '1')
        {
            Target_one();
        }

        if(c == '2')
        {
            Target_two();
        }

        if(c == 'q')
        {
            cout<<"End of patrol"<<endl;
            break;
        }

        if(c == 'h')
        {
            Target_home();
            cout<<"Mars go back to home"<<endl;
            break;
        }
    }    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "patrol_node");
    ros::NodeHandle nh;
    PatrolNode node(nh);
    ros::spin();
    
    return 0;
}