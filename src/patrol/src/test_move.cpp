#include <iostream>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <termios.h> 
#include <unistd.h> 
#include <stdio.h>  
#include <vector>
//ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h> 
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// Custom msg & srv
#include <detection_msgs/Detection2DTrig.h>
#include <detection_msgs/Det3D.h>
#include <detection_msgs/Det3DArray.h>
#include <detection_msgs/StringArray.h>


using namespace std;
geometry_msgs::Pose item_position,midlePose;
ros::Subscriber sub_item_position;
class nasal_swab
{
private:
    std_msgs::Bool grip;
    char command;
    const string PLANNING_GROUP = "tm_arm";
    const vector<double> wait_for_swab = {-0.01820257492363453, 0.1478414684534073, 2.1385350227355957, -2.4705910682678223, 1.6482404470443726, 0.013351768255233765};
    const vector<double> home_g = {-1.823, -0.4935, 1.9722, 0.085, 1.5599, 0.070};
    const vector<double> finish_g = {-1.823, -1.077, 1.9722, 0.085, 1.5599, 0.070};
    const vector<double> test = {-0.025537103414535522, -0.23823225498199463, 2.118441104888916, -1.8876608610153198, 1.69246506690979, 0.019314976409077644};
    const vector<double> home_p = {-0.017194775864481926, 0.23691341280937195, 2.100027084350586, -2.472792148590088, 1.6404544115066528, 0.012149429880082607};
    const vector<double> new_home_point = {0, -1.0395996570587158, 2.6233043670654297, -1.666411280632019, 1.6540776491165161, 0};
    float *x_tmp, *y_tmp, *z_tmp;
    int count = 0, target_amount = 0, target_number = 0;
    std_msgs::Int8 replacement_finished;
    detection_msgs::Det3DArray target_bias;
    detection_msgs::Det3D bias;
    bool find = false;
    bool grab = false;
    bool collect = false;
    bool reach = false;

public:
    nasal_swab(ros::NodeHandle nh);
    void det_callback(detection_msgs::Det3DArray msg);
    void Position_Manager();
    //void mis_callback(detection_msgs::StringArray msg);
    //void loc_callback(std_msgs::Int8 msg);
    ros::Publisher gripper_pub;
    //ros::Publisher mis_pub;
    ros::Subscriber det_sub;
    ros::Subscriber mis_sub;
    ros::Subscriber loc_sub;
    ros::Subscriber sub_item_position;
    geometry_msgs::Pose current_pose, tag_pose, ready_pose, target_pose;
    detection_msgs::StringArray mis_list, sa, prod_list;
};

nasal_swab::nasal_swab(ros::NodeHandle nh)
{  
    ros::AsyncSpinner spinner(1); 
    spinner.start();
    
    // x_tmp = new float[20] ();
    // y_tmp = new float[20] ();
    // z_tmp = new float[20] ();
    //mis_sub = nh.subscribe("/missing_bottle", 1, &ArmControl::mis_callback, this);
    gripper_pub = nh.advertise<std_msgs::Bool>("/gripper/cmd_gripper", 1);
    //mis_pub = nh.advertise<detection_msgs::Det3DArray>("/missing_bottle", 1);
    det_sub = nh.subscribe("/scan_nose_clustering_node/det3d_result", 1, &nasal_swab::det_callback, this);
    Position_Manager();
}

// base link frame
// left  +y   right -y
// front +x   back  -x
// up    +z   down  -z

int getch(void) 
{ 
    struct termios oldattr, newattr; 
    int ch; 
    tcgetattr(STDIN_FILENO, &oldattr); 
    newattr = oldattr; 
    newattr.c_lflag &= ~(ICANON | ECHO); 
    tcsetattr(STDIN_FILENO, TCSANOW, &newattr); 
    ch = getchar(); 
    tcsetattr(STDIN_FILENO, TCSANOW, &oldattr); 
    return ch; 
} 

/* reads from keypress, echoes */ 
int getche(void) 
{ 
    struct termios oldattr, newattr; 
    int ch; 
    tcgetattr(STDIN_FILENO, &oldattr); 
    newattr = oldattr; 
    newattr.c_lflag &= ~(ICANON); 
    tcsetattr(STDIN_FILENO, TCSANOW, &newattr); 
    ch = getchar(); 
    tcsetattr(STDIN_FILENO, TCSANOW, &oldattr); 
    return ch; 
} 

void nasal_swab::det_callback(detection_msgs::Det3DArray msg)
{
    if(msg.dets_list.size() != 0 && reach )
        find = true;
    else
        find = false;

    if(find && !collect)
    {
        target_bias.dets_list.clear();

        while(count < 20)
        {
            for(int i=0; i<msg.dets_list.size(); i++)
            {
                x_tmp[i] += msg.dets_list[i].x;
                y_tmp[i] += msg.dets_list[i].y;
                z_tmp[i] += msg.dets_list[i].z;
                //cout << x_tmp[i] << endl;
            }
            count++;
        }

        count = 0;

        for(int i=0; i<msg.dets_list.size(); i++)
        {
           //cout << "123"<< endl;
            //for(int j=0; j<mis_list.strings.size(); j++)
            //{
                //cout << "456"<< endl;
            //if(mis_list.strings[j] == msg.dets_list[i].class_name)
            //{
            bias.class_name = msg.dets_list[i].class_name;
            cout<<bias.class_name<<endl;
            bias.class_id = msg.dets_list[i].class_id;
            bias.x = x_tmp[i]/20;
            bias.y = y_tmp[i]/20;
            bias.z = z_tmp[i]/20;
            x_tmp[i] = 0;
            y_tmp[i] = 0;
            z_tmp[i] = 0;
            // cout << bias.x << endl;
            // cout << bias.y << endl;
            // cout << bias.z << endl;
            target_bias.dets_list.push_back(bias);    

                //}
            //}
        }
        collect = true;   
    }

    if(collect)
    {   
        target_amount = target_bias.dets_list.size();

        for(int i=0; i<target_bias.dets_list.size(); i++)
        {
            static tf::TransformBroadcaster br;
            tf::Transform tf1;
            //string tf_name1 = "tm_target_pose";
            tf1.setOrigin(tf::Vector3(target_bias.dets_list[i].x, target_bias.dets_list[i].y, target_bias.dets_list[i].z));
            tf::Quaternion q;
            q.setEuler(0.0,0.0,0.0);
            tf1.setRotation(q);
            //tf1.setRotation(tf::Quaternion(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w));
            br.sendTransform(tf::StampedTransform(tf1, ros::Time::now(), "rs_camera_link", "item_link"));
            //br.sendTransform(tf::StampedTransform(tf1, ros::Time::now(), "rs_camera_link", ""));
            string b;
            b = target_bias.dets_list[i].class_name;
            // cout << "------------------" << endl;
            // cout << target_bias.dets_list[i].x << endl;
            // cout << target_bias.dets_list[i].y << endl;
            // cout << target_bias.dets_list[i].z << endl;
            prod_list.strings.push_back(b);
        }
    }
    
    // if(collect)
    // {   
    //     target_amount = target_bias.dets_list.size();

    //     // for(int i=0; i<target_bias.dets_list.size(); i++)
    //     // {
    //     //     cout<<"test000000"<<endl;
    //     //     static tf::TransformBroadcaster br;
    //     //     tf::Transform tf1;
    //     //     string tf_name1 = "tm_target_pose";
    //     //     tf1.setOrigin(tf::Vector3(target_bias.dets_list[i].x, target_bias.dets_list[i].y, target_bias.dets_list[i].z));
    //     //     tf1.setRotation(tf::Quaternion(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w));
    //     //     br.sendTransform(tf::StampedTransform(tf1, ros::Time::now(), "camera1_link", tf_name1));

    //     //     string b;
    //     //     b = target_bias.dets_list[i].class_name;

    //     //     prod_list.strings.push_back(b);
        
    //     //     cout << target_bias.dets_list[i].x << endl;
    //     //     cout << target_bias.dets_list[i].y << endl;
    //     //     cout << target_bias.dets_list[i].z << endl;
    //     //     cout <<"-----------------------------------" <<endl;
    //     //     cout << current_pose.orientation.x << endl;
    //     //     cout << current_pose.orientation.y << endl;
    //     //     cout << current_pose.orientation.z << endl;
    //     //     cout << current_pose.orientation.w << endl;



    //     // }

    // }
}






// void nasal_swab::mis_callback(detection_msgs::StringArray msg)
// {
//     mis_list.strings = msg.strings;
//     // cout<<"0000"<<endl;
//     // cout<<"SIZE:"<<mis_list.strings.size()<<endl;
//     for(int i=0; i<msg.strings.size(); i++)
//     {
//         cout<<msg.strings[i]<<endl;
//     }
// }



// void nasal_swab::loc_callback(std_msgs::Int8 msg)
// {
//     printf("%d\n", msg.data);
//     if(msg.data == 1)
//         cout<<"At Position One"<<endl;
// }



void nasal_swab::Position_Manager()
{  
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group.setStartState(*move_group.getCurrentState());
    move_group.setMaxVelocityScalingFactor(0.1);
    //joint_group_positions = home_p2;
    //move_group.setJointValueTarget(joint_group_positions);
    //move_group.move();
    //grip.data = true;
    //gripper_pub.publish(grip);
    ROS_INFO("DONE");
    
    while(1)
    {
        command = getchar();

        if(command == 'h')
        {
            ROS_INFO("GO HOME");
            joint_group_positions = wait_for_swab;
            //move_group.setMaxVelocityScalingFactor(0.1);
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            grip.data = true;
            gripper_pub.publish(grip);
            ROS_INFO("DONE");
        }
        
        if(command == 'w')
        {
            ROS_INFO("WAIT FOR SWAB");
            joint_group_positions = wait_for_swab;
            //move_group.setMaxVelocityScalingFactor(0.1);
            move_group.setJointValueTarget(joint_group_positions);
            move_group.move();
            grip.data = true;
            gripper_pub.publish(grip);
            current_pose = move_group.getCurrentPose().pose;
            sleep(1);
            //reach = true;
            ROS_INFO("DONE");
        }


        // if(command == 'n')
        // {
        //     ROS_INFO("REACH THE NEW HOME POINT");
        //     joint_group_positions = new_home_point;
        //     //move_group.setMaxVelocityScalingFactor(0.1);
        //     move_group.setJointValueTarget(joint_group_positions);
        //     move_group.move();
        //     grip.data = false;
        //     gripper_pub.publish(grip);
        //     current_pose = move_group.getCurrentPose().pose;
        //     sleep(1);
        //     reach = true;
        //     ROS_INFO("DONE");
        // }

        if(command == 'k')
        {
            ROS_INFO("test");
            cout << "x: "<< item_position.position.x<<endl;
            cout << "y: "<< item_position.position.y<<endl;
            cout << "z: "<< item_position.position.z<<endl;
            cout << "-----------------------"<< endl;
            target_pose = current_pose;
            target_pose.position.x += (item_position.position.x-0.285);
            target_pose.position.y += (item_position.position.y + 0.05);
            target_pose.position.z += (item_position.position.z+0.18);
            move_group.setPoseTarget(target_pose);
            move_group.move();
    
            ROS_INFO("DONE");
        }

        

        if(command == 't')
        {   
            if(target_amount > 0)
            {
                for(int i=0; i<target_amount; i++)
                {
                    ROS_INFO("GO TARGET POINT");
                    
                    current_pose = move_group.getCurrentPose().pose;
                    // tf::TransformListener listener;
                    // tf::StampedTransform tf2;
                    // string tf_name2 = "/tm_target_pose";
                    // listener.waitForTransform("/tm_end_eff", tf_name2 + to_string(i), ros::Time(0), ros::Duration(4.0));
                    // listener.lookupTransform("/tm_end_eff", tf_name2 + to_string(i), ros::Time(0), tf2);
            // listener.waitForTransform("/tm_end_eff", "/tm_target_pose", ros::Time(0), ros::Duration(4.0));
            // listener.lookupTransform("/tm_end_eff", "/tm_target_pose", ros::Time(0), tf2);
                    // current_pose.position.x += tf2.getOrigin().getX();
                    // current_pose.position.y += tf2.getOrigin().getZ();
                    // current_pose.position.z += tf2.getOrigin().getY();

                    // current_pose.position.x += (target_bias.dets_list[i].x-0.325);//-0.155
                    // current_pose.position.y += (target_bias.dets_list[i].y + 0.03);//+0.08
                    // current_pose.position.z += (-target_bias.dets_list[i].z-0.07);
                    current_pose.position.x = 0.722814;
                    current_pose.position.y = -0.116816;//+0.08
                    current_pose.position.z = 0.82388;          


                    cout << target_bias.dets_list[i].x << endl;
                    cout << target_bias.dets_list[i].y << endl;
                    cout << target_bias.dets_list[i].z << endl;
                    cout << "-------------" << endl;
                    
                    cout << current_pose.position.x << endl;
                    cout << current_pose.position.y << endl;
                    cout << current_pose.position.z << endl;

                    move_group.setPoseTarget(current_pose);
                    move_group.move();

                    ROS_INFO("REACH TARGET POINT");
                }
            }        
        }




        if(command == 'q')
        {
            ROS_INFO("QUIT");
            break;
        }
    }



}

void targetPoseCallback(const geometry_msgs::Pose msg)
{
    item_position = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nasal_swab");
    ros::NodeHandle nh;
    sub_item_position = nh.subscribe("/target_pose", 1000, targetPoseCallback);
    nasal_swab node(nh);
    ros::spin();
    return 0;
}
