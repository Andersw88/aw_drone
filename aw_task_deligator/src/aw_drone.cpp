#include "aw_drone.h"



Drone::Drone ( const std::string& name ,double goalTolerance) : name_ ( name ), n_ ( name ) ,goalIsPublished_ ( false ), hasGoal_ ( false ),hasPosition_ ( false ),goalTolerance_ ( 0.5 ) //,distanceToGoal_(std::numeric_limits<float>::max())
{
    goalPub_ = n_.advertise<geometry_msgs::Point> ( "goal", 1 );
    poseSub_ = n_.subscribe<nav_msgs::Odometry> ( "ground_truth/state",1,boost::bind ( &Drone::poseCallback, this, _1 ) );
}
void Drone::publishCurrentGoal()
{
    if ( hasPosition_ && hasGoal_ && !goalIsPublished_ ) {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.pose.orientation.w = 1;
        goal.pose.position.x = currentGoal_.first;
        goal.pose.position.y = currentGoal_.second;
        goalPub_.publish ( goal );
        goalIsPublished_ = true;
        updateState();
    }


}
void Drone::publishCurrentPosAsGoal()
{
    geometry_msgs::Point goal;
//         goal.header.frame_id = "map";
//         goal.pose.orientation.w = 1;
    goal.x = position_.first;
    goal.y = position_.second;
    goalPub_.publish ( goal );



}
void Drone::setGoal ( const std::pair<double,double>& goal )
{
    currentGoal_ = goal;
    hasGoal_ = true;
    goalIsPublished_ = false;
    updateState();

}
void Drone::poseCallback ( const nav_msgs::OdometryConstPtr& pose )
{
    position_.first = pose->pose.pose.position.x;
    position_.second = pose->pose.pose.position.y;

    hasPosition_ = true;
    updateState();
}

bool Drone::isFree()
{
//     if ( hasPosition_ ) {
//         ROS_INFO ( "%s has a position",name_.c_str() );
//     } else {
//         ROS_INFO ( "%s doesn't have a position" ,name_.c_str() );
//     }
    ROS_DEBUG ( "%s state: hasPosition_:%s  hasGoal_:%s goalIsPublished_:%s",
               name_.c_str(),
               hasPosition_ ? "true":"false",
               hasGoal_ ? "true":"false",
               goalIsPublished_ ? "true":"false" );

    return ( hasPosition_ && !hasGoal_ && !goalIsPublished_ );
}
bool Drone::hasGoalAndPos()
{
    ROS_DEBUG ( "%s state: hasPosition_:%s  hasGoal_:%s goalIsPublished_:%s",
               name_.c_str(),
               hasPosition_ ? "true":"false",
               hasGoal_ ? "true":"false",
               goalIsPublished_ ? "true":"false" );
    return ( hasPosition_ && hasGoal_ );
}

bool Drone::hasPosAndMove_base()
{
    return ( hasPosition_ && ros::service::exists ( name_+"/move_base/make_plan",true ) );
}

bool Drone::calcEuclidianGoalDistances ( const std::vector< std::pair< double, double > >& goals, std::vector< double >& distances )
{
    if ( hasPosition_  && goals.size() == distances.size() ) {
        for ( int i = 0; i < goals.size(); ++i ) {
            distances[i] = ( sqrt ( pow ( ( goals[i].first - position_.first ),2 ) +
                                    pow ( ( goals[i].second - position_.second ),2 ) ) );
        }
        return true;
    } else {
        return false;
    }
}
bool Drone::getMove_baseDistances ( const std::vector< std::pair< double, double > >& goals, std::vector< double >& distances )
{
    if ( !hasPosition_  || goals.size() != distances.size() ) {
        ROS_WARN ( "!hasPosition_  || goals.size() != distances.size(): is false" );
        return false;
    }

    ros::ServiceClient make_planClient= n_.serviceClient<nav_msgs::GetPlan> ( "move_base/make_plan" );


    geometry_msgs::PoseStamped start;
    start.header.frame_id = "map";
    start.pose.orientation.w = 1;
    start.pose.position.x = position_.first;
    start.pose.position.y = position_.second;
    for ( int i = 0; i < goals.size(); ++i ) {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.pose.orientation.w = 1;
        goal.pose.position.x = goals[i].first;
        goal.pose.position.y = goals[i].second;
        nav_msgs::GetPlan srv;

        srv.request.start = start;
        srv.request.goal = goal;
        srv.request.tolerance = 0.05;

        distances[i] = 0;
        //ROS_INFO ( "Calling Make_plan:%s: goal:%d,%f,%f",name_.c_str(),i+1,goals[i].first,goals[i].second);
        if ( !make_planClient.call ( srv ) ) {
            ROS_ERROR ( "Make_plan failed:%s: goal:%d,%f,%f",name_.c_str(),i+1,goals[i].first,goals[i].second );
            return false;
        }

        std::vector<geometry_msgs::PoseStamped>& global_plan ( srv.response.plan.poses );
        if ( global_plan.size() > 0 ) {
//              sleep ( 5 );
            //ROS_DEBUG ( "after call:%s:,pos:%f,%f, goal:%d,%f,%f",name_.c_str(),position_.first,position_.second,i+1,goals[i].first,goals[i].second );
            for ( int j=0; j < global_plan.size()-1; ++j ) {
                distances[i] += ( sqrt ( pow ( ( global_plan[j+1].pose.position.x -
                                                 global_plan[j].pose.position.x ),2 ) +
                                         pow ( ( global_plan[j+1].pose.position.y -
                                                 global_plan[j].pose.position.y ),2 ) ) );
            }
            ROS_DEBUG ( "Make_plan success:%s: distnace:%f,goal:%d,%f,%f,global_plan.size()%d:",name_.c_str(), distances[i],i+1,goals[i].first,goals[i].second, ( int ) global_plan.size() );

        } else {
            ROS_ERROR ( "Make_plan failed:%s: goal:%d,%f,%f ,%d",name_.c_str(),i+1,goals[i].first,goals[i].second, ( int ) global_plan.size() );
            return false;
        }
    }
    return true;
}
const std::string& Drone::getName() const
{
    return name_;
}

// private:
void Drone::updateState()
{
    if ( hasPosition_ && hasGoal_ ) {
        distanceToGoal_ = sqrt ( pow ( ( currentGoal_.first - position_.first ),2 ) +
                                 pow ( ( currentGoal_.second - position_.second ),2 ) );

        if ( distanceToGoal_ < goalTolerance_ ) {
            hasGoal_ = false;
            goalIsPublished_ = false;
        }
    }
}

const std::pair<double,double>& Drone::getPos()
{
    return position_;
}

const std::pair<double,double>& Drone::getGoal()
{
    return currentGoal_;
}

