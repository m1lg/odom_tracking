#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

struct odom {
    double back_wheel_w;
    double back_wheel_v;
    double back_wheel_r;
};

struct odom odom_data;


void transformPoint(const tf2_ros::Buffer &buffer){
    //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
    geometry_msgs::PointStamped base_link;
    base_link.header.frame_id = "base_footprint";

    //we'll just use the most recent transform available for our simple example
    base_link.header.stamp = ros::Time();

    //just an arbitrary point in space
    base_link.point.x = 0;
    base_link.point.y = 0;
    base_link.point.z = 0;

    try{
        geometry_msgs::PointStamped base_footprint;
        buffer.transform(base_link, base_footprint, "base_link");

        ROS_INFO("base_link: (%.4f, %.4f. %.4f) -----> base_footprint: (%.4f, %.4f, %.4f) at time %.4f",
            base_link.point.x, base_link.point.y, base_link.point.z,
            base_footprint.point.x, base_footprint.point.y, base_footprint.point.z, base_footprint.header.stamp.toSec());
            odom_data.back_wheel_r  = base_footprint.point.z;
    }
    catch(tf2::TransformException &ex){
        ROS_ERROR("Received an exception trying to transform a point from \"base_footprint\" to \"base_link\": %s", ex.what());
    }
}


void counterCallbackJoint(const sensor_msgs::JointState::ConstPtr& msg) {// Define a function called 'callback' that receives 
                                                                         // a parameter named 'msg' 
	std::vector<std::string> names = msg->name;
	if (msg->name[0][0] == 'f') {
	
        ROS_INFO("%s", &names[0][0]);
        odom_data.back_wheel_v = (msg->velocity[1]+msg->velocity[2])/2;
		ROS_INFO("avg= %f",odom_data.back_wheel_v);
	} else {
		
        ROS_INFO("%s", &names[0][0]);
	}
}

void counterCallbackIMU(const sensor_msgs::Imu::ConstPtr& msg) {
    ROS_INFO("IMU subscribe");
}

void counterCallbackGPS(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    ROS_INFO("GPS subscribe");
}


int main(int argc, char** argv) {
    	
	bool tf2Received = 0;
    ros::init(argc, argv, "odom_tracking"); // Initiate a Node called 'odom_tracking'
    ros::NodeHandle nh;

    tf2_ros::Buffer tf2Buffer;
    tf2_ros::TransformListener tf2Listener(tf2Buffer);
    while(!tf2Received) {

        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tf2Buffer.lookupTransform("base_link", "base_footprint",
            ros::Time::now());//, ros::Duration(10.0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        tf2Received = 1;
    }
    
    transformPoint(tf2Buffer);
    
    ros::Subscriber sub_joint_states = nh.subscribe("/zio/joint_states", 1000, counterCallbackJoint);
    ros::Subscriber sub_imu = nh.subscribe("/vn100/imu", 1000, counterCallbackIMU);
    ros::Subscriber sub_gps = nh.subscribe("/ublox_gps/fix", 1000, counterCallbackGPS);
    // Create a Subscriber object that will listen 
    // to the /counter topic and will call the 
    // 'callback' function each time it reads 
    // somethin from the topic
   
    ros::Rate loop_rate(50); // Set a publish rate of 2 Hz

    while (ros::ok()) {
    
        //ROS_INFO("Wheel radius = %f", odom_data.back_wheel_r);
        ros::spinOnce();
        loop_rate.sleep(); // Make sure the publish rate maintains at 2 Hz
    }
    return 0;

    //ros::spin(); // Create a loop that will keep the program in execution
    
    }
