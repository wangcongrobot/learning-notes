# tf

http://wiki.ros.org/tf

tf si a package that lets the user keep track of multiple coordinate frames over time. tf maintains the relationship between coordinates frames in a tree structure buffered in time, and lets the user transform points, vectors, etc between any two coordinate frames at any desired point in time.


## code
https://blog.csdn.net/weixin_40799950/article/details/83620820

```cpp

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// #include "std_msgs/String.h"
#include <object_recognition_msgs/RecognizedObjectArray.h> //using ORK
#include <visualization_msgs/MarkerArray.h> 
 
//实时转换采集到的可乐罐的位置点转换
 
std::string Object_id = "";
double Object_assurance = 0;
geometry_msgs::PoseStamped Object_pose;
geometry_msgs::PoseStamped transed_pose;
bool firstCB = false;
 
void myCallback(const object_recognition_msgs::RecognizedObjectArray objects_msg) //回调函数 
{ 
  double confident = 0;
  int id = -1;
  
  if (firstCB == false && (int)objects_msg.objects.size() == 1) {
        Object_id.assign(objects_msg.objects[0].type.key.c_str());
        firstCB = true;
    }
  
  for (int i = 0; i < objects_msg.objects.size(); ++i) {
    if (Object_id.compare(objects_msg.objects[i].type.key.c_str()) == 0) {
      if (objects_msg.objects[i].confidence > confident) {
          
          confident = objects_msg.objects[i].confidence;
          id = i;
          }
        }
    }
 
  if (id >= 0) {
        Object_pose.pose = objects_msg.objects[id].pose.pose.pose;
        Object_assurance = objects_msg.objects[id].confidence;
    } else {
        confident = 0;
    }
}
 
void transformPose(const tf::TransformListener& listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the  frame
  // geometry_msgs::PointStamped laser_point;
  // geometry_msgs::PoseStamped transed_pose;
  // laser_point.header.frame_id = "camera_link";
  // msg.header.frame_id = "camera_link";
 
  // Object_pose.header.frame_id = "camera_rgb_frame";
  Object_pose.header.frame_id = "camera_rgb_optical_frame";
  //we'll just use the most recent transform available for our simple example
  // laser_point.header.stamp = ros::Time();
  ROS_INFO("Waiting for the object");
  if (Object_assurance > 0.8) {
      ROS_INFO("Best Similarity = %f ", Object_assurance);
      ROS_INFO("pose x is: %f", Object_pose.pose.position.x);
      ROS_INFO("pose y is: %f", Object_pose.pose.position.y);
      ROS_INFO("pose z is: %f", Object_pose.pose.position.z);
 
      bool tferr = true;
      while (tferr) {
        tferr = false;
      
      try{  
        listener.transformPose("base_link", Object_pose, transed_pose);
        }
      catch (tf::TransformException &exception) {
          ROS_ERROR("Received an exception trying to transform a point from \"camera_rgb_frame\" to \"base_link\": %s", exception.what());
          tferr = true;
          ros::Duration(0.1).sleep(); 
          ros::spinOnce();
        }
 
      } //while
 
    ROS_INFO("camera_rgb_optical_frame: (%.2f, %.2f. %.2f, %.2f, %.2f, %.2f, %.2f) -----> base_link: (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f) at time %.2f",
        Object_pose.pose.position.x, Object_pose.pose.position.y, Object_pose.pose.position.z, Object_pose.pose.orientation.x, Object_pose.pose.orientation.y, Object_pose.pose.orientation.z, Object_pose.pose.orientation.w,
        transed_pose.pose.position.x, transed_pose.pose.position.y, transed_pose.pose.position.z, transed_pose.pose.orientation.x, transed_pose.pose.orientation.y, transed_pose.pose.orientation.z, transed_pose.pose.orientation.w, transed_pose.header.stamp.toSec());
  }  //if
} //void
 
int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;
 
  ros::Subscriber my_subscriber_object= n.subscribe("recognized_object_array",1000,myCallback);
  tf::TransformListener listener(ros::Duration(10));
 
  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPose, boost::ref(listener)));
  ros::Publisher my_publisher_transPose = n.advertise<geometry_msgs::PoseStamped>("transformPose", 1000);
 
  ros::Publisher my_publisher_ObjectPose = n.advertise<geometry_msgs::PoseStamped>("ObjectPose", 1000);
 
  ros::Rate loop_rate(10);
  while (ros::ok())
    {
      // std_msgs::String msg;
      // msg.data = ss.str(); 
      // ROS_INFO("=====transformedPose======: (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", transed_pose.pose.position.x, transed_pose.pose.position.y, transed_pose.pose.position.z, transed_pose.pose.orientation.x, transed_pose.pose.orientation.y, transed_pose.pose.orientation.z, transed_pose.pose.orientation.w);
      my_publisher_transPose.publish(transed_pose);
      my_publisher_ObjectPose.publish(Object_pose);
 
      ros::spinOnce();
      loop_rate.sleep();
    }
    ros::spin();
    return 0;
}
```
————————————————
版权声明：本文为CSDN博主「羊羊羊机器人」的原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/weixin_40799950/article/details/83620820

