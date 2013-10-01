#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <math.h>

#include <sstream>
class Navigator
{
  ros::NodeHandle n;
  geometry_msgs::Twist velocity_;
  ros::Subscriber poseSub_;
  ros::Subscriber goalSub_;
  ros::Publisher cmdvelPub_;
  geometry_msgs::Point goal_;
  bool hasGoal_;

public:
  Navigator() : hasGoal_(false){
    poseSub_ = n.subscribe<nav_msgs::Odometry>("ground_truth/state", 1000, boost::bind(&Navigator::poseCallback, this, _1));
    goalSub_ = n.subscribe<geometry_msgs::Point>("goal", 1000, boost::bind(&Navigator::goalCallback, this, _1));
    cmdvelPub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  }

  void publish()
  {
	if(hasGoal_) cmdvelPub_.publish(velocity_);
  }

  void poseCallback(const nav_msgs::OdometryConstPtr& pose)
  {
    // geometry_msgs::PoseWithCovariance pose2 =  pose->pose.pose.position;
    geometry_msgs::Point tpoint = pose->pose.pose.position;
    // geometry_msgs::Point trotation = pose->pose.pose.orientation;
    // geometry_msgs::Twist ttwist = pose->twist.twist;
    geometry_msgs::Twist temp_out_twist;
    // velocity_. = geometry_msgs::Twist(geometry_msgs::Vector3(1.0,1.0,1.0),geometry_msgs::Vector3());
    velocity_.linear.x = std::min((goal_.x - tpoint.x),2.0);// * Math::cos(costrotation.z);
    velocity_.linear.y = std::min((goal_.y - tpoint.y),2.0);
    velocity_.linear.z = 0;//(goal_.z - tpoint.z);
	velocity_.angular.z = -tf::getYaw(pose->pose.pose.orientation);
    
//     ROS_INFO("I heard: pose2");
  }
    void goalCallback(const geometry_msgs::PointConstPtr& goal)
  {
    goal_ = *goal;
	hasGoal_ = true;
//     ROS_INFO("I heard: goal");
  }
};

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "aw_2d_nav");


  Navigator nav;

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
    {

      std_msgs::String msg;

      std::stringstream ss;
//       ss << "hello world " << count;
      msg.data = ss.str();

//       ROS_INFO("%s", msg.data.c_str());
	
      nav.publish();

      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }


  return 0;
}
