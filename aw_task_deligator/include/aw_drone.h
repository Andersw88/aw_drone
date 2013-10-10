#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetPlan.h"
#include "gazebo_msgs/SetModelState.h"

#include <math.h>

#ifndef AW_DRONE_QQQ
#define AW_DRONE_QQQ

class Drone
{
    // geometry_msgs::PoseStamped currentGoal_;
    std::string name_;
    ros::NodeHandle n_;
    ros::Publisher goalPub_;
    ros::Subscriber poseSub_;
    std::pair<double,double> position_;
    std::pair<double,double> currentGoal_;
    float distanceToGoal_;
    bool hasGoal_;
    bool hasPosition_;
	bool goalIsPublished_;
	double goalTolerance_;

public:

    Drone ( const std::string& name ,double goalTolerance = 0.5);
    void publishCurrentGoal();
    void setGoal ( const std::pair<double,double>& goal );

    bool calcEuclidianGoalDistances ( const std::vector< std::pair< double, double > >& goals, std::vector< double >& distances );
    void poseCallback ( const nav_msgs::OdometryConstPtr& pose );

    bool isFree();
	bool hasGoalAndPos();
	bool hasPosAndMove_base();
    const std::string& getName() const;
    bool getMove_baseDistances ( const std::vector< std::pair< double, double > >& goals, std::vector< double >& distances );
	void publishCurrentPosAsGoal();
	
	bool setGazeboPos(std::pair<double,double> pos);
    const std::pair<double,double>& getPos();
	const std::pair<double,double>& getGoal();

private:
    void updateState();

};


#endif
