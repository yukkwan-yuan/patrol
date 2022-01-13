#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
using namespace std;

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);

void cloudCB (const sensor_msgs::PointCloud2ConstPtr input)
{
    pcl::fromROSMsg (*input, *cloud_msg);
}

void u_v_CB(const std_msgs::Int16MultiArray input){
    int u=input.data[0];
    int v=input.data[1];
    int i = u+v*cloud_msg->width;
    if(isnan(cloud_msg->points[i].x)){

    }
    else{
        float x3Ddepth = cloud_msg->points[i].x;
        float y3Ddepth = cloud_msg->points[i].y;
        float z3Ddepth = cloud_msg->points[i].z;
        geometry_msgs::Pose item_position;
        item_position.position.x = z3Ddepth;
        item_position.position.y = -x3Ddepth;
        item_position.position.z = -y3Ddepth;
        cout <<"x: " << item_position.position.x << endl;
        cout <<"y: " << item_position.position.y << endl;
        cout <<"z: " << item_position.position.z << endl;
        cout <<"-----------------------------------------"<<endl;
        //tf_broadcaster
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(item_position.position.x,item_position.position.y,item_position.position.z));
        tf::Quaternion q;
        q.setEuler(0.0,0.0,0.0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"rs_camera_link", "item_link"));

        pub.publish(item_position);
    }
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "pointcloud2xyz");
  ros::NodeHandle nh;
  ros::Subscriber coordinate_sub =nh.subscribe("target_xy", 1, u_v_CB);
  ros::Subscriber cloud_sub =nh.subscribe("/ready_depth_could", 1, cloudCB);
  pub = nh.advertise<geometry_msgs::Pose> ("target_pose", 1);
  ros::spin ();
  return 0;
}
