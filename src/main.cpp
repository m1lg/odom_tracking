#include <ros/ros.h>
#include <string>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>

struct odom {
    double back_wheel_vel;    
};

struct odom odom_data;

void counterCallbackJoint(const sensor_msgs::JointState::ConstPtr& msg) {// Define a function called 'callback' that receives 
                                                                         // a parameter named 'msg' 
	std::vector<std::string> names = msg->name;
	if (msg->name[0][0] == 'f') {
	
        ROS_INFO("%s", &names[0][0]);
        odom_data.back_wheel_vel = (msg->velocity[1]+msg->velocity[2])/2;
		ROS_INFO("avg= %f",odom_data.back_wheel_vel);
	} else {
		
        ROS_INFO("%s", &names[0][0]);
	}
}

void counterCallbackIMU(const sensor_msgs::Imu::ConstPtr& msg) {
    ROS_INFO("IMU subscribe");
}


int main(int argc, char** argv) {
    	
	ros::init(argc, argv, "topic_subscriber"); // Initiate a Node called 'topic_subscriber'
	ros::NodeHandle nh;
    
    ros::Subscriber sub_joint_states = nh.subscribe("/zio/joint_states", 1000, counterCallbackJoint); 
    ros::Subscriber sub_imu = nh.subscribe("/vn100/imu", 1000, counterCallbackIMU);   
                                                // Create a Subscriber object that will listen 
    										    // to the /counter topic and will call the 
										        // 'callback' function each time it reads 
										        // somethin from the topic

    tf::TransformListener tf_listen;

    ros::spin(); // Create a loop that will keep the program in execution
    
    return 0;
}
