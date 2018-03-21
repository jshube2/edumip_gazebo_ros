#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "edumip_msgs/EduMipState.h"
#include "tf/transform_broadcaster.h"

#define PI 3.1415926535
#define RADIUS 0.031
class edumipViz
{
  public:

    edumipViz()
    {
      ros::NodeHandle n;

      sub = n.subscribe<edumip_msgs::EduMipState>("/edumip/state", 1000, &edumipViz::callback, this);

      pub_j = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);

      pub_o = n.advertise<nav_msgs::Odometry>("/edumip/odometry", 1000);  

    };


  private:

    ros::Publisher pub_j;
    ros::Publisher pub_o;
    ros::Subscriber sub;  
    sensor_msgs::JointState joint_state;

    void callback(const edumip_msgs::EduMipState::ConstPtr& edu_state)
    { 
      // Grab data from edumip
      float body_x = edu_state->body_frame_northing;
      float body_y = -edu_state->body_frame_easting;
      float body_h = -edu_state->body_frame_heading;
      float body_t = edu_state->theta;

      ros::Time current_time = ros::Time::now();

      // Populate joint state message
      joint_state.header.stamp = current_time;
      joint_state.name.resize(2);
      joint_state.position.resize(2);
      joint_state.name[0] ="jointL";
      joint_state.position[0] = edu_state->wheel_angle_L;
      joint_state.name[1] ="jointR";
      joint_state.position[1] = edu_state->wheel_angle_R;
      pub_j.publish(joint_state);

      // Publish transform from world to edumip_body
      static tf::TransformBroadcaster tfbc;  
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(body_x, body_y, RADIUS) );
      tf::Quaternion q;
      q.setRPY(0, body_t, body_h);
      transform.setRotation(q);
      tfbc.sendTransform(tf::StampedTransform(transform, current_time, "world", "edumip_body"));

      // Publish odometry message that tracks the motion of edumip_body
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(body_h);

      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "world";
      odom.child_frame_id = "odometry";
      
      odom.pose.pose.position.x = body_x;
      odom.pose.pose.position.y = body_y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;
      pub_o.publish(odom);
    }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "edumipViz");
  edumipViz e;
  ros::spin();

  return 0;

}
