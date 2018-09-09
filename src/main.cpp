#include <ros/ros.h>
#include <time.h>
#include <string>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

struct imu {
    double roll;
    double pitch;
    double yaw;
};

struct dim {
  double wheel_r;
  double wheel_base;
  double wheel_track;
};


struct joint {
    double back_wheel_v1;
    double back_wheel_v2;
    ros::Time time1;
    ros::Time time2;
    ros::Duration dt;
};

struct odom {
    struct imu degree;
    struct imu rad;
    tf::Quaternion odom_quat_tf;
    double dx;
    double dy;
    double x;
    double y;
    double z;
    double v;
    //time stamp;
};

struct odom odometry;
struct joint sensor_data;
struct dim vehicle;

ros::Publisher odom_pub;

void jointTransforms(const tf2_ros::Buffer &buffer){
    //create points for dimensional transforms of car
    geometry_msgs::PointStamped base_link;
    geometry_msgs::PointStamped base_footprint;
    geometry_msgs::PointStamped rear_right_wheel_link;
    geometry_msgs::PointStamped rear_left_wheel_link;
    geometry_msgs::PointStamped right_steering_link;


    base_link.header.frame_id = "base_link";
    base_link.header.stamp = ros::Time();
    
    
    //just an arbitrary point in space
    base_link.point.x = 0;
    base_link.point.y = 0;
    base_link.point.z = 0;
    
    base_footprint.header.frame_id = "base_footprint";
    buffer.transform(base_link, base_footprint, "base_footprint");
    vehicle.wheel_r = base_footprint.point.z;              //base footprint is on the ground, the transform from baselink will give the wheel radius measurement
    

    rear_right_wheel_link.header.frame_id = "rear_right_wheel_link";
    buffer.transform(base_link, rear_right_wheel_link, "rear_right_wheel_link");
    vehicle.wheel_track = 2*rear_right_wheel_link.point.y;    //the transform from baselink to the rear right wheel link is half the measurement of the track of the car
    

    right_steering_link.header.frame_id = "right_steering_link";
    buffer.transform(rear_right_wheel_link, right_steering_link, "right_steering_link");
    vehicle.wheel_base = -1*right_steering_link.point.x;       //the transform from the rear right wheel to the right steering link will give us the wheel base measurment in the x direction
        

    
    rear_left_wheel_link.header.frame_id = "rear_left_wheel_link";
    buffer.transform(base_link, rear_left_wheel_link, "rear_left_wheel_link");

}


void calculateOdom(void){
    
    
    
    //static tf::TransformBroadcaster odom_broadcaster;
    
    
    odometry.dx = (odometry.v*sin(odometry.rad.yaw)*sensor_data.dt.toSec());
    odometry.dy = (odometry.v*cos(odometry.rad.yaw)*sensor_data.dt.toSec());
    odometry.x += odometry.dx;
    odometry.y += odometry.dy;
    
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(odometry.odom_quat_tf, odom_quat);


    //tf::Transform transform;
    //transform.setOrigin( tf::Vector3(odometry.x, odometry.y, 0.0) );
    //tf::Quaternion q(0,0,0,1);
    //q.setEuler(0, 0, odom_quat);
    //transform.setRotation(q);
    //odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "testing"));
    
    
    
    //geometry_msgs::TransformStamped odom_transform;
    //odom_transform.header.stamp = sensor_data.time2;
    //odom_transform.header.frame_id = "tfbroadcast";
    //odom_transform.child_frame_id = "base_link";
    
    //odom_transform.transform.translation.x = odometry.x;
    //odom_transform.transform.translation.y = odometry.y;
    //odom_transform.transform.translation.z = odometry.z;
    //odom_transform.transform.rotation = odom_quat;
    
    
    //odom_broadcaster.sendTransform(odom_transform);
    
    nav_msgs::Odometry odom;
    odom.header.stamp = sensor_data.time2;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    
    
    odom.pose.pose.position.x = odometry.x;
    odom.pose.pose.position.y = odometry.y;
    odom.pose.pose.position.z = odometry.z;
    odom.pose.pose.orientation = odom_quat;
    
    odom.twist.twist.linear.x = sin(odometry.rad.yaw);
    odom.twist.twist.linear.y = cos(odometry.rad.yaw);
    
    odom_pub.publish(odom);
    
    
    
    ROS_INFO("x = %f\t y = %f\t", odometry.dx, odometry.dy);
}

//callback for handling the motion data from the joint states
void counterCallbackJoint(const sensor_msgs::JointState::ConstPtr& msg) {// Define a function called 'callback' that receives a parameter named 'msg' 
                                                                       
    std::vector<std::string> names = msg->name;	
    if (msg->name[0][0] == 'f') {                              //if the first letter of the first name in the array is f then we have the velocity data
        double angular_v;
        angular_v = (msg->velocity[1]+msg->velocity[2])/2;     //centre of vehicle/baselink velocity wheel be the average of the two rear wheel velocities
        sensor_data.back_wheel_v1 = sensor_data.back_wheel_v2;
        sensor_data.back_wheel_v2 = angular_v*vehicle.wheel_r;  //linear velocity = angular velocity * wheel radius
        odometry.v =  (sensor_data.back_wheel_v1 + sensor_data.back_wheel_v2)/2;
        sensor_data.time1 = sensor_data.time2;                 //previous time stored
        sensor_data.time2 = msg->header.stamp;                 //save current time from sensor header time stamp
        sensor_data.dt = sensor_data.time2 - sensor_data.time1;  //dt is the difference between current time and previous time 
        
        if ((sensor_data.dt.toSec() < 1)&&(sensor_data.dt.toSec() > 0)){   //first time is un initialised and > than 1, this ensures we disregard that
      
            //ROS_INFO("dt = %f", sensor_data.dt.toSec());  //showing change of time           
            calculateOdom();
            
        } else if (sensor_data.dt.toSec() < 0){     // When bag file loops, dt is < 0, 
            ROS_INFO("End of bag file reached");  // End program when end of bag file is reached
            exit(0);
        }
        
	} 
}

//Callback for handling the IMU data input
void counterCallbackIMU(const sensor_msgs::Imu::ConstPtr& msg) {

    

    tf::quaternionMsgToTF(msg->orientation, odometry.odom_quat_tf);  
    tf::Matrix3x3(odometry.odom_quat_tf).getRPY(odometry.rad.roll, odometry.rad.pitch , odometry.rad.yaw);
    

        
    odometry.degree.roll = odometry.rad.roll*(180/M_PI);    //converting radians to degrees
    odometry.degree.pitch = odometry.rad.pitch*(180/M_PI);
    odometry.degree.yaw = odometry.rad.yaw*(180/M_PI);
    

    //ROS_INFO("yaw = %f", sensor_data.current.y);
}

//callback for handling the GPS data input
void counterCallbackGPS(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    //ROS_INFO("GPS subscribe");
}

//main node for handling the sequence of the programs execution
int main(int argc, char** argv) {
    	
	bool tf2Received = 0;
	sensor_data.back_wheel_v2 = 0;
    ros::init(argc, argv, "odom_tracking"); // Initiate a Node called 'odom_tracking'
    ros::NodeHandle nh;                     // initiate a node handler

    tf2_ros::Buffer tf2Buffer;              //create a buffer for handling transform data
    tf2_ros::TransformListener tf2Listener(tf2Buffer);  //listen for the transform data 
    
    while(!tf2Received) {                     //wait until transform data has started transmitting to proceed
    
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tf2Buffer.lookupTransform("base_link", "base_footprint",
            ros::Time::now());//, ros::Duration(10.0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());           //warn data hasnt been recieved 
            ros::Duration(1.0).sleep();
            continue;
        }
        tf2Received = 1;
    }
    
    ROS_INFO("Bag file started");
    jointTransforms(tf2Buffer); //run static transforms with the transform data
    
    ros::Subscriber sub_joint_states = nh.subscribe("/zio/joint_states", 1000, counterCallbackJoint);   //create a subscriber that listens for the joint states data
    ros::Subscriber sub_imu = nh.subscribe("/vn100/imu", 1000, counterCallbackIMU); //create a subscriber that listens for the IMU data
    ros::Subscriber sub_gps = nh.subscribe("/ublox_gps/fix", 1000, counterCallbackGPS); //create a subscriber that listens for the gps data
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    
   
    ros::Rate loop_rate(50); // Set a publish rate of 2 Hz

    while (ros::ok()) {
    
        //ROS_INFO("Wheel radius = %f", odom_data.back_wheel_r);
        ros::spinOnce();
        loop_rate.sleep(); // Make sure the publish rate maintains at 2 Hz
    }
    return 0;

    //ros::spin(); // Create a loop that will keep the program in execution
    
    }
