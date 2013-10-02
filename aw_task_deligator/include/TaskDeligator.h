#include "ros/ros.h"
#include "std_msgs/String.h"
#include "aw_task_deligator/Objectives.h"
#include "aw_task_deligator/getObjectives.h"
#include <math.h>
#include <sstream>
#include "aw_drone.h"
#include "munkres.h"
#include "threshold.h"


#ifndef AW_TASK_DELIGATOR_QQQ
#define AW_TASK_DELIGATOR_QQQ

class TaskDeligator
{
protected:
    std::vector<std::pair<double,double> > tasks_;
    std::vector<boost::shared_ptr<Drone> > drones_;
public:
    TaskDeligator ( std::vector< boost::shared_ptr< Drone > > drones, std::vector< std::pair< double, double > > tasks );
    virtual bool allocTasksToDrones() = 0;
    void publishGoals();
	
	bool getObjectivesSrv( aw_task_deligator::getObjectivesRequest& req, aw_task_deligator::getObjectivesResponse& resp);
};


class RandomTask : public TaskDeligator
{
public:
    RandomTask ( std::vector< boost::shared_ptr< Drone > > drones, std::vector< std::pair< double, double > > tasks );
    virtual bool allocTasksToDrones();
};


class HungarianTask : public TaskDeligator
{
public:
    HungarianTask ( std::vector< boost::shared_ptr< Drone > > drones, std::vector< std::pair< double, double > > tasks );
    virtual bool allocTasksToDrones();
};


class ThresholdPlusHungarianTask : public TaskDeligator
{
public:
    ThresholdPlusHungarianTask ( std::vector< boost::shared_ptr< Drone > > drones, std::vector< std::pair< double, double > > tasks );
    virtual bool allocTasksToDrones();
};
#endif
