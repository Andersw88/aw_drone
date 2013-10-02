#include "TaskDeligator.h"


TaskDeligator::TaskDeligator ( std::vector<boost::shared_ptr<Drone> > drones,std::vector<std::pair<double,double> > tasks ) : drones_ ( drones ),tasks_ ( tasks ) {}
void TaskDeligator::publishGoals()
{
    for ( int i = 0; i<drones_.size(); ++i ) {
        drones_[i]->publishCurrentGoal();
    }
}

bool TaskDeligator::getObjectivesSrv ( aw_task_deligator::getObjectivesRequest& req, aw_task_deligator::getObjectivesResponse& resp )
{
    geometry_msgs::Point start;
    geometry_msgs::Point target;
    for ( int i = 0; i<drones_.size(); ++i ) {

        if ( !drones_[i]->hasGoalAndPos() ) {
            return false;
        }

        start.x = drones_[i]->getPos().first;
        start.y = drones_[i]->getPos().second;

        target.x = drones_[i]->getGoal().first;
        target.y = drones_[i]->getGoal().second;

//         objs.starts.push_back ( geometry_msgs::Point(drones_[i]->getPos().first,drones_[i]->getPos().second) );
//         objs.targets.push_back ( geometry_msgs::Point(drones_[i]->getGoal().first,drones_[i]->getGoal().second));
        resp.objectives.starts.push_back ( start );
        resp.objectives.targets.push_back ( target );
    }
//resp.objectives = objs;
    return true;
}




RandomTask::RandomTask ( std::vector<boost::shared_ptr<Drone> > drones,std::vector<std::pair<double,double> > tasks ) : TaskDeligator ( drones,tasks ) {}
bool RandomTask::allocTasksToDrones()
{
    for ( int i = 0; i<drones_.size(); ++i ) {
        // drones_[i]->updateGoalDist();
        if ( !drones_[i]->isFree() ) {
            return false;
        }
    }

    for ( int i = 0; i<drones_.size(); ++i ) {
        ROS_INFO ( "New task for %s, num tasks left %d", drones_[i]->getName().c_str(), ( int ) tasks_.size() );
        if ( tasks_.size() == 0 ) {
            ROS_WARN ( "Not enough tasks for all drones" );
            return false;
        }
        int taskNr = rand() % tasks_.size();
        drones_[i]->setGoal ( tasks_[taskNr] );
        tasks_.erase ( tasks_.begin() +taskNr );
    }

    return true;

}

HungarianTask::HungarianTask ( std::vector<boost::shared_ptr<Drone> > drones,std::vector<std::pair<double,double> > tasks ) : TaskDeligator ( drones,tasks ) {}

bool isInvalidGoal ( std::pair<double,double> p )
{
    return ( p.first < 0 && p.second < 0 );
}

bool HungarianTask::allocTasksToDrones()
{
    int nrDrones = drones_.size();
    int nrTasks = tasks_.size();
    if ( nrTasks == 0 ) {
        ROS_DEBUG ( "Already done,all tasks are have been completed" );
        return false;
    }

    for ( int i = 0; i<nrDrones; ++i ) {
        if ( !drones_[i]->isFree() ) {
            ROS_DEBUG ( "Drone:%d is busy",i+1 );
            return false;
        }
    }

    std::vector<std::vector<double> >  distances ( nrDrones,std::vector<double> ( nrTasks ) );
    ROS_INFO ( "Running task deligation with %d drones and %d tasks Step 1",nrDrones,nrTasks );
    for ( int i = 0; i<nrDrones; ++i ) {

//         if ( !drones_[i]->calcEuclidianGoalDistances ( tasks_,distances[i] ) ) {
//             ROS_INFO ( "Drone:%d can not provide distances",i+1 );
//             return false;
//         }
        if ( !drones_[i]->getMove_baseDistances ( tasks_,distances[i] ) ) {
            ROS_INFO ( "Drone:%d can not provide distances",i+1 );
            return false;
        }
    }


    Matrix<double> distancesMatrix ( nrDrones,nrTasks );

    for ( int i = 0; i < nrDrones; ++i ) {
        for ( int j = 0; j < nrTasks; ++j ) {
            distancesMatrix ( i,j ) = distances[i][j];
        }
    }

    std::stringstream ss;
    ss.precision ( 0 );
    ss << "Resulting matrix:";
    for ( int row = 0 ; row < nrDrones ; row++ ) {
        ss << std::endl;
        for ( int col = 0 ; col < nrTasks ; col++ ) {
            ss.width ( 5 );
            ss << std::fixed << distancesMatrix ( row,col ) << ",";
        }
    }
    ss << std::endl;

    Munkres m;
    m.solve ( distancesMatrix );

    ss << std::endl;
    ss << "Solved matrix:";
    for ( int row = 0 ; row < nrDrones ; row++ ) {
        ss << std::endl;
        for ( int col = 0 ; col < nrTasks ; col++ ) {
            ss.width ( 5 );
            ss << distancesMatrix ( row,col ) << ",";
        }
    }
    ss << std::endl;
    ROS_INFO ( "%s",ss.str().c_str() );

    for ( int i = 0; i < nrDrones; ++i ) {
        for ( int j = 0; j < nrTasks; ++j ) {
            if ( !distancesMatrix ( i,j ) ) {
                drones_[i]->setGoal ( tasks_[j] );
                tasks_[j] = std::make_pair ( -1,-1 );
            }
        }
    }
    tasks_.erase ( std::remove_if ( tasks_.begin(), tasks_.end(), isInvalidGoal ),tasks_.end() );


    return true;
}

ThresholdPlusHungarianTask::ThresholdPlusHungarianTask ( std::vector<boost::shared_ptr<Drone> > drones,std::vector<std::pair<double,double> > tasks ) : TaskDeligator ( drones,tasks ) {}

bool ThresholdPlusHungarianTask::allocTasksToDrones()
{
    int nrDrones = drones_.size();
    int nrTasks = tasks_.size();
    if ( nrTasks == 0 ) {
        ROS_DEBUG ( "Already done,all tasks are have been completed" );
        return false;
    }

    for ( int i = 0; i<nrDrones; ++i ) {
        if ( !drones_[i]->isFree() ) {
            ROS_DEBUG ( "Drone:%d is busy",i+1 );
            return false;
        }
    }

    std::vector<std::vector<double> >  distances ( nrDrones,std::vector<double> ( nrTasks ) );
    ROS_INFO ( "Running task deligation with %d drones and %d tasks Step 1",nrDrones,nrTasks );
    for ( int i = 0; i<nrDrones; ++i ) {

//         if ( !drones_[i]->calcEuclidianGoalDistances ( tasks_,distances[i] ) ) {
//             ROS_INFO ( "Drone:%d can not provide distances",i+1 );
//             return false;
//         }
        if ( !drones_[i]->getMove_baseDistances ( tasks_,distances[i] ) ) {
            ROS_INFO ( "Drone:%d can not provide distances",i+1 );
            return false;
        }
    }


    Matrix<double> distancesMatrix ( nrDrones,nrTasks );

    for ( int i = 0; i < nrDrones; ++i ) {
        for ( int j = 0; j < nrTasks; ++j ) {
            distancesMatrix ( i,j ) = distances[i][j];
        }
    }

    std::stringstream ss;
    ss.precision ( 0 );
    ss << "Resulting matrix:";
    for ( int row = 0 ; row < nrDrones ; row++ ) {
        ss << std::endl;
        for ( int col = 0 ; col < nrTasks ; col++ ) {
            ss.width ( 5 );
            ss << std::fixed << distancesMatrix ( row,col ) << ",";
        }
    }
    ss << std::endl;
	ThresholdAlgorithm ta;
	ta.solve(distancesMatrix);
	
	ss << std::endl;
    ss << "Threshold matrix:";
    for ( int row = 0 ; row < nrDrones ; row++ ) {
        ss << std::endl;
        for ( int col = 0 ; col < nrTasks ; col++ ) {
            ss.width ( 5 );
            ss << distancesMatrix ( row,col ) << ",";
        }
    }
	
    Munkres m;
    m.solve ( distancesMatrix );

    ss << std::endl;
    ss << "Solved matrix:";
    for ( int row = 0 ; row < nrDrones ; row++ ) {
        ss << std::endl;
        for ( int col = 0 ; col < nrTasks ; col++ ) {
            ss.width ( 5 );
            ss << distancesMatrix ( row,col ) << ",";
        }
    }
    ss << std::endl;
    ROS_INFO ( "%s",ss.str().c_str() );

    for ( int i = 0; i < nrDrones; ++i ) {
        for ( int j = 0; j < nrTasks; ++j ) {
            if ( !distancesMatrix ( i,j ) ) {
                drones_[i]->setGoal ( tasks_[j] );
                tasks_[j] = std::make_pair ( -1,-1 );
            }
        }
    }
    tasks_.erase ( std::remove_if ( tasks_.begin(), tasks_.end(), isInvalidGoal ),tasks_.end() );
    return true;
}







