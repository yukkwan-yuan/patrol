#include <iostream>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

//ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// Custom msg & srv
#include <detection_msgs/Detection2DTrig.h>
#include <detection_msgs/Det3D.h>
#include <detection_msgs/Det3DArray.h>

using namespace std;

class warehouse_action
{
private:
    std_msgs::Bool grip;
    char target_number, last_target_number = '0';
    const string PLANNING_GROUP = "tm_arm";
    const vector<double> home_p = {-M_PI_2, -M_PI_4, M_PI*2/3, -1.309, M_PI, 0.0};
    const vector<double> home_r = {M_PI, M_PI/3, -M_PI_4*3, 1.309, -M_PI_2, 0.0};
    const vector<double> middle1_a = {-M_PI, -M_PI_4, M_PI*2/3, -1.309, M_PI, 0.000};
    const vector<double> middle1_b = {-2.552, -0.527, 2.218, -1.679, 1.009, 0.000};
    const vector<double> middle2_a = {0, -M_PI_4, M_PI*2/3, -1.309, M_PI, 0.000};
    const vector<double> middle2_b = {3.344, -1.023, 2.259, -1.218, 1.285, 0.000};
    const vector<double> middler_p = {M_PI_2, -M_PI/3, M_PI_4*3, -1.309, -M_PI_2, 0.0};
    const vector<double> middlep_r = {0.000, 0.000, 0.000, 0.000, -M_PI_2, 0.0};
    const vector<double> position_s = {0, M_PI/3, -M_PI_4*3, 1.309, 0.000, 0.000}; //M_PI_4*3

    const vector<double> joint_sf_scan1 = {-1.564, 0.014, 2.261, -2.227, 1.550, 0.000};
    const vector<double> joint_sf_scan2 = {-0.870, 0.014, 2.161, -2.185, 0.888, 0.000};
    const vector<double> joint_sf_scan3 = {-0.305, -0.485, 1.473, -1.953, 0.286, 0.000};

    const vector<double> joint_wh_scan1 = {-0.305, 0.485, 1.635, -1.953, 0.293, 0.160};
    const vector<double> joint_wh_scan2 = {-0.772, -0.048, 2.320, -2.200, 0.758, 0.056};
    const vector<double> joint_wh_scan3 = {-1.564, 0.014, 2.261, -2.227, 1.549, 0.003};
    const vector<double> joint_wh_scan4 = {-2.176, 0.384, 1.793, -2.118, 2.154, -0.032};


    const vector<double> joint_place1 = {-3.133, 0.180, 2.279, -2.450, 1.545, 0.000};
    const vector<double> joint_place1_mid = {-3.133, -0.282, 2.255, -2.234, 1.547, 0.002};
    const vector<double> joint_place2 = {-2.583, 0.183, 2.462, -2.644, 1.019, 0.000};
    const vector<double> joint_place2_mid = {-2.822, -0.355, 2.231, -2.045, 1.473, 0.016};
    const vector<double> joint_place3 = {-0.020, 0.269, -1.979, -1.416, 1.551, 3.130};
    const vector<double> joint_place3_mid = {-0.028, 0.410, -1.616, -1.813, 1.544, 3.041};
    const vector<double> joint_place4 = {3.181, -0.120,2.709, -2.586, 1.454, 0.034};
    const vector<double> joint_place4_mid = {3.344, -1.023, 2.259, -1.218, 1.285, 0.028};
    const vector<double> joint_place5 = {0.012, 0.647, -2.194, -1.579, 1.552, 3.040};
    const vector<double> joint_place5_mid = {0.002, 0.755, -1.787, -2.089, 1.533, 3.035};

    float *x_tmp, *y_tmp, *z_tmp;
    int count = 0;
    bool find = false, grab = false, collect = false, reach = false;
    detection_msgs::Det3DArray target_bias;
    detection_msgs::Det3D bias;
public:
    warehouse_action(ros::NodeHandle nh);
    void det_callback(detection_msgs::Det3DArray msg);
    void Position_Manager();

    ros::Publisher gripper_pub;
    ros::Subscriber det_sub;
    geometry_msgs::Pose current_pose;
};

warehouse_action::warehouse_action(ros::NodeHandle nh)
{  
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    x_tmp = new float[10] ();
    y_tmp = new float[10] ();
    z_tmp = new float[10] ();

    gripper_pub = nh.advertise<std_msgs::Bool>("/gripper/cmd_gripper", 1);
    det_sub = nh.subscribe("/scan_clustering_node/det3d_result", 1, &warehouse_action::det_callback, this);
    Position_Manager();
}

void warehouse_action::det_callback(detection_msgs::Det3DArray msg)
{
    if(msg.dets_list.size() != 0 && reach)
        find = true;
    else
        find = false;

    if(find && !collect)
    {
        while(count < 10)
        {
            for(int i=0; i<msg.dets_list.size(); i++)
            {
                x_tmp[i] += msg.dets_list[i].y;
                y_tmp[i] += msg.dets_list[i].x;
                z_tmp[i] += msg.dets_list[i].z;
            }
            count++;
        }

        for(int i=0; i<msg.dets_list.size(); i++)
        {
            bias.class_name = msg.dets_list[i].class_name;
            bias.class_id = msg.dets_list[i].class_id;
            bias.x = x_tmp[i]/10;
            bias.y = y_tmp[i]/10;
            bias.z = z_tmp[i]/10;
            target_bias.dets_list.push_back(bias);
        }
        collect = true;
    }

    if(collect)
    {
        for(int i=0; i<target_bias.dets_list.size(); i++)
        {
            static tf::TransformBroadcaster br;
            tf::Transform tf1;
            tf1.setOrigin(tf::Vector3(target_bias.dets_list[i].x, target_bias.dets_list[i].z, target_bias.dets_list[i].y));
            tf1.setRotation(tf::Quaternion(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w));
            br.sendTransform(tf::StampedTransform(tf1, ros::Time::now(), "camera1_link", "tm_target_pose"));
        }
    }
}

void warehouse_action::Position_Manager()
{
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group.setStartState(*move_group.getCurrentState());
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success;

    // target_pose.position.x = 0.280;  //(-)camera right
    // target_pose.position.y = -0.200; //(-)camera forward
    // target_pose.position.z = 0.800;  //(+)camera upward
    // target_pose.position.x += 0.148 + 0.03;     //target_bias.position.y
    // target_pose.position.y -= 0.100;       //target_bias.position.x
    // target_pose.position.z += -0.048;      //target_bias.position.z
    // move_group.setPoseTarget(target_pose1);
    // move_group.move();
    // target_pose1.position.y -= 0.138 - 0.008;
    // move_group.setPoseTarget(target_pose1);
    // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // move_group.move();

    // tf::TransformListener listener;
    // tf::StampedTransform tf1;
    // listener.waitForTransform("/base_link", "/camera1_link", ros::Time(0), ros::Duration(4.0));
    // listener.lookupTransform("/base_link", "/camera1_link", ros::Time(0), tf1);
    // cout<<"Current Pose"<<endl;
    // cout<<target_pose.position.x<<endl;
    // cout<<target_pose.position.y<<endl;
    // cout<<target_pose.position.z<<endl;
    // cout<<"Relative Pose"<<endl;
    // cout<<tf1.getOrigin().getX()<<endl;
    // cout<<tf1.getOrigin().getY()<<endl;
    // cout<<tf1.getOrigin().getZ()<<endl;

    // if(0)
    // {
    //     static tf::TransformBroadcaster br;
    //     tf::Transform tf2;
    //     tf2.setOrigin(tf::Vector3(0.2, 0.2, 0.2));
    //     tf2.setRotation(tf::Quaternion(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w));
    //     br.sendTransform(tf::StampedTransform(tf2, ros::Time::now(), "camera1_link", "tm_target_pose"));

    //     tf::StampedTransform tf3;
    //     listener.waitForTransform("/tm_end_eff", "/tm_target_pose", ros::Time(0), ros::Duration(4.0));
    //     listener.lookupTransform("/tm_end_eff", "/tm_target_pose", ros::Time(0), tf3);

    //     cout<<"Target Pose"<<endl;
    //     cout<<tf3.getOrigin().getX()<<endl;
    //     cout<<tf3.getOrigin().getY()<<endl;
    //     cout<<tf3.getOrigin().getZ()<<endl;
    // }

    while(1)
    {
        target_number = getchar();

        if(target_number == 'h')
        {
            ROS_INFO("GO HOME");

            if(last_target_number == '4')
            {
                joint_group_positions = middle2_b;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();

                joint_group_positions = middle2_a;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();

                joint_group_positions = home_p;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();
            }
            else if(last_target_number == '1' || last_target_number == '2' || last_target_number == '3' || last_target_number == '5')
            {
                joint_group_positions = middle1_b;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();

                joint_group_positions = home_p;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();
            }
            else
            {
                joint_group_positions = home_p;
                move_group.setJointValueTarget(joint_group_positions);
                move_group.move();
            }
            last_target_number = target_number;

            ROS_INFO("DONE");
        }
        if(target_number == '1')
        {
            grip.data = true;
            gripper_pub.publish(grip);
            sleep(1);

            // ROS_INFO("GO SCAN 3");
            // joint_group_positions = joint_wh_scan3;
            // move_group.setJointValueTarget(joint_group_positions);
            // move_group.move();            

            ROS_INFO("GO MIDDLE POINT");

            joint_group_positions = joint_place1_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO PLACE 1");
            joint_group_positions = joint_place1;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            
            grip.data = false;
            gripper_pub.publish(grip);

            ROS_INFO("GO MIDDLE POINT");
            joint_group_positions = joint_place1_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            joint_group_positions = joint_place1_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            joint_group_positions = joint_place1_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO SCAN 3");
            joint_group_positions = joint_wh_scan3;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("DONE");

            last_target_number = target_number;
        }
        if(target_number == '2')
        {
            grip.data = true;
            gripper_pub.publish(grip);
            sleep(1);
            
            ROS_INFO("GO SCAN 3");
            joint_group_positions = joint_wh_scan3;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();            

            ROS_INFO("GO MIDDLE POINT");

            joint_group_positions = joint_place2_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO PLACE 2");
            joint_group_positions = joint_place2;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            
            grip.data = false;
            gripper_pub.publish(grip);

            ROS_INFO("GO MIDDLE POINT");
            joint_group_positions = joint_place2_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            joint_group_positions = joint_place2_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            joint_group_positions = joint_place2_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO SCAN 3");
            joint_group_positions = joint_wh_scan3;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("DONE");

            last_target_number = target_number;
        }
        if(target_number == '3')
        {   
            grip.data = true;
            gripper_pub.publish(grip);
            sleep(1);

            ROS_INFO("GO SCAN 3");
            joint_group_positions = joint_wh_scan3;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            
            ROS_INFO("GO MIDDLE POINT");

            joint_group_positions = joint_place3_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO PLACE 3");
            joint_group_positions = joint_place3;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            
            grip.data = false;
            gripper_pub.publish(grip);

            ROS_INFO("GO MIDDLE POINT");
            joint_group_positions = joint_place3_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            joint_group_positions = joint_place3_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            joint_group_positions = joint_place3_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO SCAN 3");
            joint_group_positions = joint_wh_scan3;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("DONE");

            last_target_number = target_number;
        }
        if(target_number == '4')
        {   
            grip.data = true;
            gripper_pub.publish(grip);
            sleep(1);
            
            ROS_INFO("GO SCAN 3");
            joint_group_positions = joint_wh_scan3;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();            

            ROS_INFO("GO MIDDLE POINT");

            joint_group_positions = joint_place4_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO PLACE 1");
            joint_group_positions = joint_place4;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            
            grip.data = false;
            gripper_pub.publish(grip);

            ROS_INFO("GO MIDDLE POINT");
            joint_group_positions = joint_place4_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            joint_group_positions = joint_place4_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            joint_group_positions = joint_place4_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO SCAN 3");
            joint_group_positions = joint_wh_scan3;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("DONE");

            last_target_number = target_number;
        }
        if(target_number == '5')
        {
            grip.data = true;
            gripper_pub.publish(grip);
            sleep(1);
            
            ROS_INFO("GO SCAN 3");
            joint_group_positions = joint_wh_scan3;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();            

            ROS_INFO("GO MIDDLE POINT");

            joint_group_positions = joint_place5_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO PLACE 1");
            joint_group_positions = joint_place5;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            
            grip.data = false;
            gripper_pub.publish(grip);

            ROS_INFO("GO MIDDLE POINT");
            joint_group_positions = joint_place5_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            joint_group_positions = joint_place5_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            joint_group_positions = joint_place5_mid;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO SCAN 3");
            joint_group_positions = joint_wh_scan3;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("DONE");

            last_target_number = target_number;
        }
        if(target_number == 's')
        {
            ROS_INFO("GO SCANNING POINT 1");
            
            joint_group_positions = joint_sf_scan1;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO SCANNING POINT 2");

            joint_group_positions = joint_sf_scan2;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO SCANNING POINT 3");

            joint_group_positions = joint_sf_scan3;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            last_target_number = target_number;

            ROS_INFO("DONE");
        }
        if(target_number == 'w')
        {
            ROS_INFO("GO SCANNING POINT 1");
            
            joint_group_positions = joint_wh_scan1;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO SCANNING POINT 2");

            joint_group_positions = joint_wh_scan2;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();

            ROS_INFO("GO SCANNING POINT 3");

            joint_group_positions = joint_wh_scan3;
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            current_pose = move_group.getCurrentPose().pose;
            reach = true;
            
            sleep(3);
            ROS_INFO("DONE");

            tf::TransformListener listener;
            tf::StampedTransform tf2;
            listener.waitForTransform("/tm_end_eff", "/tm_target_pose", ros::Time(0), ros::Duration(4.0));
            listener.lookupTransform("/tm_end_eff", "/tm_target_pose", ros::Time(0), tf2);

            cout<<"Relative Pose"<<endl;
            cout<<tf2.getOrigin().getX()<<endl;
            cout<<tf2.getOrigin().getY()<<endl;
            cout<<tf2.getOrigin().getZ()<<endl;

            current_pose.position.x += tf2.getOrigin().getX();
            current_pose.position.y -= tf2.getOrigin().getZ()-0.1;
            current_pose.position.z += tf2.getOrigin().getY();

            move_group.setPoseTarget(current_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();

            current_pose.position.y -= 0.1;

            move_group.setPoseTarget(current_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            move_group.move();

            last_target_number = target_number;
        }
        if(target_number == 'q')
        {
            ROS_INFO("QUIT");
            break;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "warehouse_action");
    ros::NodeHandle nh;
    warehouse_action node(nh);
    ros::spin();
    return 0;
}