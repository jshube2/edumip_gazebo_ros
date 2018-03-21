#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

class JoyTwist
{
  public:
    geometry_msgs::Twist twist;

    JoyTwist()
    {
      ros::NodeHandle n;

      sub = n.subscribe<sensor_msgs::Joy>("/joy", 1000, &JoyTwist::joyCallback, this);

      pub = n.advertise<geometry_msgs::Twist>("/edumip/cmd", 1000);  

      timer = n.createTimer(ros::Duration(0.1), &JoyTwist::timerCallback, this);
    };


  private:

    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Timer timer;

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
    { 
      twist.linear.x = joy->axes[1];
      twist.linear.y = 0.0;
      twist.linear.z = 0.0;
      twist.angular.x = 0.0;
      twist.angular.y = 0.0;
      twist.angular.z = joy->axes[0];
    }

    void timerCallback(const ros::TimerEvent& event)
    {
      pub.publish(twist);
    }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  JoyTwist j;
  ros::spin();

  return 0;

}
