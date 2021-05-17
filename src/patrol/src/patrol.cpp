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
    void Target_three();
    void Target_four();
    void Target_five();
    void Target_six();
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
    while(!ac.waitForServer(ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 7.288;
    goal.target_pose.pose.position.y = -1.459;
    goal.target_pose.pose.position.z = 0.000;
    goal.target_pose.pose.orientation.x = 0.000;
    goal.target_pose.pose.orientation.y = 0.000;
    goal.target_pose.pose.orientation.z = 0.850;
    goal.target_pose.pose.orientation.w = 0.526;

    ROS_INFO("Sending Goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the goal");
    else
        ROS_INFO("Sending Next Goal");
}

void PatrolNode::Target_two()
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 3.942;
    goal.target_pose.pose.position.y = 1.675;
    goal.target_pose.pose.position.z = 0.000;
    goal.target_pose.pose.orientation.x = 0.000;
    goal.target_pose.pose.orientation.y = 0.000;
    goal.target_pose.pose.orientation.z = 0.984;
    goal.target_pose.pose.orientation.w = -0.179;

    ROS_INFO("Sending Goal");
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(10.0));

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the goal");
    else
        ROS_INFO("Sending Next Goal");
}

void PatrolNode::Target_three()
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.878;
    goal.target_pose.pose.position.y = 0.275;
    goal.target_pose.pose.position.z = 0.000;
    goal.target_pose.pose.orientation.x = 0.000;
    goal.target_pose.pose.orientation.y = 0.000;
    goal.target_pose.pose.orientation.z = -0.508;
    goal.target_pose.pose.orientation.w = 0.861;

    ROS_INFO("Sending Goal");
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(5.0));

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the goal");
    else
        ROS_INFO("Sending Next Goal");
}

void PatrolNode::Target_four()
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 4.368;
    goal.target_pose.pose.position.y = -4.045;
    goal.target_pose.pose.position.z = 0.000;
    goal.target_pose.pose.orientation.x = 0.000;
    goal.target_pose.pose.orientation.y = 0.000;
    goal.target_pose.pose.orientation.z = -0.523;
    goal.target_pose.pose.orientation.w = 0.852;

    ROS_INFO("Sending Goal");
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(25.0));

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the goal");
    else
        ROS_INFO("Sending Next Goal");
}

void PatrolNode::Target_five()
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 7.470;
    goal.target_pose.pose.position.y = -5.639;
    goal.target_pose.pose.position.z = 0.000;
    goal.target_pose.pose.orientation.x = 0.000;
    goal.target_pose.pose.orientation.y = 0.000;
    goal.target_pose.pose.orientation.z = 0.246;
    goal.target_pose.pose.orientation.w = 0.969;

    ROS_INFO("Sending Goal");
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(10.0));

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the goal");
    else
        ROS_INFO("Sending Next Goal");
}

void PatrolNode::Target_six()
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 8.215;
    goal.target_pose.pose.position.y = -4.161;
    goal.target_pose.pose.position.z = 0.000;
    goal.target_pose.pose.orientation.x = 0.000;
    goal.target_pose.pose.orientation.y = 0.000;
    goal.target_pose.pose.orientation.z = 0.746;
    goal.target_pose.pose.orientation.w = 0.666;

    ROS_INFO("Sending Goal");
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(6.0));

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the goal");
    else
        ROS_INFO("Sending Next Goal");
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

        if(c == '3')
        {
            Target_three();
        }

        if(c == '4')
        {
            Target_four();
        }

        if(c == '5')
        {
            Target_five();
        }

        if(c == '6')
        {
            Target_six();
        }

        if(c == 's')
        {
            Target_two();
            Target_three();
            Target_four();
            Target_five();
            Target_six();
            Target_one();
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