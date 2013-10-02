#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <XmlRpcException.h>
#include <math.h>
#include <limits>
#include <complex>
#include "aw_drone.h"
#include "TaskDeligator.h"




int main ( int argc, char **argv )
{

    ros::init ( argc, argv, "aw_task_deligator" );
    srand ( time ( NULL ) );

    ros::NodeHandle n;
	
	int deligatorMode = -1;
	double goalTolerance = 0.5;
	
    XmlRpc::XmlRpcValue Xdrones;
    XmlRpc::XmlRpcValue Xgoals;
    XmlRpc::XmlRpcValue xDeligatorMode;
	XmlRpc::XmlRpcValue xGoalTolerance;

    n.getParam ( "drones", Xdrones );
    n.getParam ( "goals", Xgoals );
    n.getParam ( "aw/goal_deligator_mode", xDeligatorMode );
	n.getParam ( "aw/goal_tolerance", xGoalTolerance );
	
    if ( !Xdrones.valid() ) {
        ROS_FATAL ( "Invalid drones param" );
        return ( -1 );
    }
    if ( !Xgoals.valid() ) {
        ROS_FATAL ( "Invalid goals param" );
        return ( -1 );
    }
    if ( xDeligatorMode.valid() && xDeligatorMode.getType() ==  XmlRpc::XmlRpcValue::TypeInt) {
        deligatorMode  = static_cast<int> ( xDeligatorMode );
    } else {
        ROS_WARN ( "Invalid aw/goal_deligator_mode param. Defaulting to Hungarian" );
        deligatorMode = 0;
    }
    
	if ( xGoalTolerance.valid() && xGoalTolerance.getType() ==  XmlRpc::XmlRpcValue::TypeDouble) {
		goalTolerance  = static_cast<double> ( xGoalTolerance );
    } else {
		ROS_WARN ( "Invalid aw/goal_tolerance param. Defaulting to 0.5" );
        goalTolerance = 0.5;
    }

    std::vector<std::pair<double,double> > tasks;
    for ( int i = 0; i < Xgoals.size(); i++ ) {
        tasks.push_back ( std::make_pair ( static_cast<double> ( Xgoals[i][0] ),static_cast<double> ( Xgoals[i][1] ) ) );
    }

    std::vector<boost::shared_ptr<Drone> > drones;
    for ( int i = 0; i < Xdrones.size(); i++ ) {
        drones.push_back ( boost::make_shared<Drone> ( static_cast<std::string> ( Xdrones[i] ) ) );
        ROS_INFO ( "Found drone name :%s", drones[i]->getName().c_str() );
    }

    // TaskAllocator* taskAlloc =  new RandomTaskAllocator(drones,tasks);
//     RandomTask taskAlloc ( drones,tasks );
    ros::Rate loop_rate ( 0.15 );
    bool dronesHasPosAndMove_base = false;
    while ( !dronesHasPosAndMove_base ) { //Makes Sure that all drones are initalized.
        ros::spinOnce();
        dronesHasPosAndMove_base = true;
        for ( int i = 0; i< drones.size(); ++i ) {
            dronesHasPosAndMove_base = drones[i]->hasPosAndMove_base(); 
            if ( !dronesHasPosAndMove_base ) {
                ROS_INFO_NAMED ( "aw_task_deligator","Drone%d not ready yet. Waiting",i+1 );
                break;
            }
        }
        sleep ( 1 );
    }
    for ( int i = 0; i< drones.size(); ++i ) {
        drones[i]->publishCurrentPosAsGoal();
    }

	TaskDeligator* taskAlloc = NULL;
    switch ( deligatorMode ) {
    case 0:
        taskAlloc = new HungarianTask ( drones,tasks );
        break;
    case 1:
        taskAlloc = new RandomTask ( drones,tasks );
        break;
	case 2:
        taskAlloc = new ThresholdPlusHungarianTask ( drones,tasks );
        break;
    default:
        ROS_WARN ( "Deligator mode %d is not a valid mode. Hungarian = 0, Random = 1. Defaulting to Hungarian", deligatorMode );
        taskAlloc = new HungarianTask ( drones,tasks );
        break;
    }

    while ( !taskAlloc->allocTasksToDrones() ) { //Makes sure that goals have successfully been allocated before providing getObjectives service.
        ros::spinOnce();
        sleep ( 5 );
    }

    ros::ServiceServer service = n.advertiseService ( "getObjectives", &TaskDeligator::getObjectivesSrv,taskAlloc );
    ROS_INFO ( "aw_task_deligator: Providing service \"\\getObjectives\"." );


    while ( ros::ok() ) {
        ros::spinOnce();
        taskAlloc->allocTasksToDrones();
        loop_rate.sleep();
    }

    delete taskAlloc;
    return 0;
}


