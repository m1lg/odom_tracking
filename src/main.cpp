#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>

void counterCallback(const nav_msgs::Odometry::ConstPtr& msg)  // Define a function called 'callback' that receives a                                                                
                                                            // parameter named 'msg' 
{
  ROS_INFO("x = %f\t y = %f\t z = %f\t", msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z); // Print the value 'data' inside the 'msg' parameter
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "topic_subscriber"); // Initiate a Node called 'topic_subscriber'
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("odom", 1000, counterCallback);   // Create a Subscriber object that will                                                                               
                                                                            // listen to the /counter topic and will
                                                                            // call the 'callback' function each time                                                                             // it reads something from the topic
    
    ros::spin(); // Create a loop that will keep the program in execution
    
    return 0;
}
