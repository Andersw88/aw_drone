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

        std::pair<double,double> pos = drones_[i]->getPos();
        std::pair<double,double> goal = drones_[i]->getGoal();
        if ( !drones_[i]->hasGoalAndPos() ) {
            ROS_WARN ( "%s,failed on hasGoalAndPos dronePos:%f,%f;goal:%f,%f, Drone might already be at goal",drones_[i]->getName().c_str(),pos.first,pos.second,goal.first,goal.second );

        }

        start.x = pos.first;
        start.y = pos.second;

        target.x = goal.first;
        target.y = goal.second;

//         objs.starts.push_back ( geometry_msgs::Point(drones_[i]->getPos().first,drones_[i]->getPos().second) );
//         objs.targets.push_back ( geometry_msgs::Point(drones_[i]->getGoal().first,drones_[i]->getGoal().second));
        resp.objectives.starts.push_back ( start );
        resp.objectives.targets.push_back ( target );
    }
//resp.objectives = objs;
    return true;
}

bool isInvalidGoal ( std::pair<double,double> p )
{
    return ( p.first < 0 && p.second < 0 );
}

bool TaskDeligator::allocTasksToDrones()
{
    int nrDrones = drones_.size();
    int nrTasks = tasks_.size();

    for ( int i = 0; i<drones_.size(); ++i ) {
        if ( !drones_[i]->isFree() ) {
            return false;
        }
    }
    std::vector<std::vector<double> >  distances ( nrDrones,std::vector<double> ( nrTasks ) );


    if ( !getDistances ( distances ) ) {
        return false;
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

    solveMatrix ( distancesMatrix );


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



bool TaskDeligator::getDistances ( std::vector<std::vector<double> >&  distances )
{

// 	std::vector<std::vector<double> >  distances ( nrDrones,std::vector<double> ( nrTasks ) );
    int nrDrones = distances.size();
    int nrTasks = distances[0].size();
    ROS_INFO ( "Running task deligation with %d drones and %d tasks Step 1",nrDrones,nrTasks );
    for ( int i = 0; i<nrDrones; ++i ) {
        if ( !drones_[i]->getMove_baseDistances ( tasks_,distances[i] ) ) {
            ROS_INFO ( "Drone:%d can not provide distances",i+1 );
            return false;
        }
    }
    return true;
}

// RandomTask::RandomTask ( std::vector<boost::shared_ptr<Drone> > drones,std::vector<std::pair<double,double> > tasks ) : TaskDeligator ( drones,tasks ) {}
// bool RandomTask::allocTasksToDrones()
// {
//     for ( int i = 0; i<drones_.size(); ++i ) {
//         // drones_[i]->updateGoalDist();
//         if ( !drones_[i]->isFree() ) {
//             return false;
//         }
//     }
//
//     for ( int i = 0; i<drones_.size(); ++i ) {
//         ROS_INFO ( "New task for %s, num tasks left %d", drones_[i]->getName().c_str(), ( int ) tasks_.size() );
//         if ( tasks_.size() == 0 ) {
//             ROS_WARN ( "Not enough tasks for all drones" );
//             return false;
//         }
//         int taskNr = rand() % tasks_.size();
//         drones_[i]->setGoal ( tasks_[taskNr] );
//         tasks_.erase ( tasks_.begin() +taskNr );
//     }
//
//     return true;
//
// }

HungarianTask::HungarianTask ( std::vector<boost::shared_ptr<Drone> > drones,std::vector<std::pair<double,double> > tasks ) : TaskDeligator ( drones,tasks ) {}

void HungarianTask::solveMatrix ( Matrix<double>& distancesMatrix )
{
    Munkres m;
    m.solve ( distancesMatrix );
}


ThresholdPlusHungarianTask::ThresholdPlusHungarianTask ( std::vector<boost::shared_ptr<Drone> > drones,std::vector<std::pair<double,double> > tasks ) : TaskDeligator ( drones,tasks ) {}

void ThresholdPlusHungarianTask::solveMatrix ( Matrix<double>& distancesMatrix )
{
    ThresholdAlgorithm ta;
    ta.solve ( distancesMatrix );
    Munkres m;
    m.solve ( distancesMatrix );
}

GreedyFirst::GreedyFirst ( std::vector<boost::shared_ptr<Drone> > drones,std::vector<std::pair<double,double> > tasks ) : TaskDeligator ( drones,tasks ) {}

void GreedyFirst::solveMatrix ( Matrix<double>& distancesMatrix )
{
    int nrDrones = distancesMatrix.rows();
    int nrTasks = distancesMatrix.columns();
    assert ( nrTasks == nrDrones );
    std::vector<bool> taskMask ( nrTasks,false );
    for ( int i = 0; i < nrDrones; ++i ) {
        int minValue = 999999;
        int task = -1;
        for ( int j = 0; j < nrTasks; ++j ) {
            if ( distancesMatrix ( i,j ) < minValue && !taskMask[j] ) {
                task = j;
                minValue = distancesMatrix ( i,j );
            }
        }
        if ( minValue < 999999 && task != -1 ) {
            taskMask[task] = true;
            distancesMatrix ( i,task ) = 0;
        } else {
            ROS_ERROR ( "GreedyFirst assignment failed, no minValue found" );
        }
    }
}

GreedyFirstv2::GreedyFirstv2 ( std::vector<boost::shared_ptr<Drone> > drones,std::vector<std::pair<double,double> > tasks ) : TaskDeligator ( drones,tasks ) {}

void GreedyFirstv2::solveMatrix ( Matrix<double>& distancesMatrix )
{
    int nrDrones = distancesMatrix.rows();
    int nrTasks = distancesMatrix.columns();
    assert ( nrTasks == nrDrones );

    std::vector<std::pair<int,int> > droneOrderIndex;
    int minValue = 999999;

    for ( int i = 0; i < nrDrones; ++i ) {
        minValue = 999999;
        for ( int j = 0; j < nrTasks; ++j ) {
            if ( distancesMatrix ( i,j ) < minValue ) {
                minValue=distancesMatrix ( i,j );
            }
        }
        droneOrderIndex.push_back ( std::make_pair<int,int> ( minValue,i ) );
    }
    std::sort ( droneOrderIndex.begin(),droneOrderIndex.end() );

    std::vector<bool> taskMask ( nrTasks,false );
    for ( int i = 0; i < nrDrones; ++i ) {
        int currDrone = droneOrderIndex[i].second;
        int minValue = 999999;
        int task = -1;
        for ( int j = 0; j < nrTasks; ++j ) {
            if ( distancesMatrix ( currDrone,j ) < minValue && !taskMask[j] ) {
                task = j;
                minValue = distancesMatrix ( currDrone,j );
            }
        }
        if ( minValue < 999999 && task != -1 ) {
            taskMask[task] = true;
            distancesMatrix ( currDrone,task ) = 0;
        } else {
            ROS_ERROR ( "GreedyFirst assignment failed, no minValue found" );
        }
    }
}
