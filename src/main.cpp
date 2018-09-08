#include <ros/ros.h>
#include <time.h>
#include <string>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>

struct imu {
    double r;
    double p;
    double y;
};

struct dim {
  double wheel_r;
  double wheel_base;
  double wheel_track;
};


struct joint {
    ros::Time time1;
    ros::Time time2;
    ros::Duration dt;
    double back_wheel_old;
    double back_wheel_v;
    struct imu inital;
    struct imu current;
    bool init = 1;
};

struct odom {
    
    //time stamp;


};


struct joint sensor_data;
struct dim vehicle;

void staticTransform(const tf2_ros::Buffer &buffer){
    //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
    geometry_msgs::PointStamped base_link;
    geometry_msgs::PointStamped base_footprint;
    geometry_msgs::PointStamped rear_right_wheel_link;
    geometry_msgs::PointStamped rear_left_wheel_link;
    geometry_msgs::PointStamped right_steering_link;

    base_link.header.frame_id = "base_link";
    //We'll just use the most recent transform available for our simple example
    base_link.header.stamp = ros::Time();

    //just an arbitrary point in space
    base_link.point.x = 0;
    base_link.point.y = 0;
    base_link.point.z = 0;

    try{
        base_footprint.header.frame_id = "base_footprint";

        buffer.transform(base_link, base_footprint, "base_footprint");
        vehicle.wheel_r = base_footprint.point.z;
        //ROS_INFO("base_link: (%.4f, %.4f. %.4f) -----> base_footprint: (%.4f, %.4f, %.4f) at time %.4f",
        //        base_link.point.x, base_link.point.y, base_link.point.z,
        //        base_footprint.point.x, base_footprint.point.y, base_footprint.point.z, base_footprint.header.stamp.toSec());
    }
    catch(tf2::TransformException &ex){
        ROS_ERROR("Received an exception trying to transform a point from \"base_link\" to \"base_footprint\": %s", ex.what());
    }
    
    try{
        rear_right_wheel_link.header.frame_id = "rear_right_wheel_link";

        buffer.transform(base_link, rear_right_wheel_link, "rear_right_wheel_link");
        vehicle.wheel_track = 2*rear_right_wheel_link.point.y;
        //ROS_INFO("base_link: (%.4f, %.4f. %.4f) -----> rear_right_wheel_link: (%.4f, %.4f, %.4f) at time %.4f",
        //        base_link.point.x, base_link.point.y, base_link.point.z, rear_right_wheel_link.point.x, 
        //        rear_right_wheel_link.point.y, rear_right_wheel_link.point.z, rear_right_wheel_link.header.stamp.toSec());
    }
    catch(tf2::TransformException &ex){
        ROS_ERROR("Received an exception trying to transform a point from \"base_link\" to \"rear_right_wheel_link\": %s", 
                ex.what());
    }
    

    try{
        right_steering_link.header.frame_id = "right_steering_link";

        buffer.transform(rear_right_wheel_link, right_steering_link, "right_steering_link");
        vehicle.wheel_base = -1*right_steering_link.point.x;
        //ROS_INFO("rear_right_link: (%.4f, %.4f. %.4f) -----> right_steering_link: (%.4f, %.4f, %.4f) at time %.4f",
        //        rear_right_wheel_link.point.x, rear_right_wheel_link.point.y, rear_right_wheel_link.point.z,
        //        right_steering_link.point.x, right_steering_link.point.y, right_steering_link.point.z, 
        //        right_steering_link.header.stamp.toSec());
    }
    catch(tf2::TransformException &ex){
        ROS_ERROR("Received an exception trying to transform a point from \"rear_right_wheel_link\" to"  
                "\"right_steering_link\": %s", ex.what());
    }

    //try{
    //    rear_left_wheel_link.header.frame_id = "rear_left_wheel_link";

    //    buffer.transform(base_link, rear_left_wheel_link, "rear_left_wheel_link");

    //    ROS_INFO("base_link: (%.4f, %.4f. %.4f) -----> rear_left_wheel_link: (%.4f, %.4f, %.4f) at time %.4f",
    //            base_link.point.x, base_link.point.y, base_link.point.z, rear_left_wheel_link.point.x, 
    //            rear_left_wheel_link.point.y, rear_left_wheel_link.point.z, rear_left_wheel_link.header.stamp.toSec());
    //}
    //catch(tf2::TransformException &ex){
    //    ROS_ERROR("Received an exception trying to transform a point from \"rear_left_wheel_link\" to \"base_link\": %s", 
    //            ex.what());
    //}

}


void counterCallbackJoint(const sensor_msgs::JointState::ConstPtr& msg) {// Define a function called 'callback' that receives 
                                                                         // a parameter named 'msg' 
    
    std::vector<std::string> names = msg->name;	
    if (msg->name[0][0] == 'f') {
        double angular_v;
        angular_v = (msg->velocity[1]+msg->velocity[2])/2;
        sensor_data.back_wheel_v = angular_v*vehicle.wheel_r;	
        sensor_data.time1 = sensor_data.time2;
        sensor_data.time2 = msg->header.stamp;
        sensor_data.dt = sensor_data.time2 - sensor_data.time1;
        ROS_INFO("dt = %f", sensor_data.dt.toSec());

	} else {
		
        //ROS_INFO("%s", &names[0][0]);
	}
}

void counterCallbackIMU(const sensor_msgs::Imu::ConstPtr& msg) {
    
    double roll, pitch, yaw;
    tf::Quaternion q;

    tf::quaternionMsgToTF(msg->orientation,q);  
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    if (sensor_data.init) {
        
        sensor_data.inital.r = roll; 
        sensor_data.inital.p = pitch;
        sensor_data.inital.y = yaw;
        sensor_data.init = 0;
    } else {
        
        sensor_data.current.r = roll;
        sensor_data.current.p = pitch;
        sensor_data.current.y = yaw;
    } 
}

void counterCallbackGPS(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    //ROS_INFO("GPS subscribe");
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
    
    staticTransform(tf2Buffer);
    
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
