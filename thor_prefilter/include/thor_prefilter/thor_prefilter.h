#ifndef THOR_PREFILTER____
#define THOR_PREFILTER____

#include <trajectory_msgs/JointTrajectory.h>
#include <ros/console.h>
namespace thor
{

class ThorPrefilter
{
protected:
  trajectory_msgs::JointTrajectoryPtr m_trj;
  unsigned int m_order;
public:
  ThorPrefilter();
  bool setTrajectory(const trajectory_msgs::JointTrajectoryPtr& trj);
  void setSplineOrder(const unsigned int& order);
  bool interpolate(const ros::Duration& time, trajectory_msgs::JointTrajectoryPoint& pnt, const double& scaling=1.0);
  ros::Duration trjTime();
};
  
}













#endif