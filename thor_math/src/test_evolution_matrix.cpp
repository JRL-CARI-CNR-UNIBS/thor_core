#include <thor_math/thor_math.h>
#include <ros/ros.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "test_evolution_matrix");
  ros::NodeHandle nh;
  
  unsigned nax=3;
  
  ROS_INFO_STREAM("Free matrix in t=3\n" << thor::math::freeResponse(3,nax).matrix());
  ROS_INFO_STREAM("Forced matrix in t=3\n" << thor::math::forcedResponse(3,nax).matrix());
  
  
  double control_horizon=0.1;
  unsigned int nc=3;
  double st=0.01;
  Eigen::VectorXd control_intervals(nc);
  Eigen::VectorXd prediction_time(nc); 
  
  thor::math::quadraticControlIntervals(control_horizon,nc,st,control_intervals,prediction_time);
  ROS_INFO_STREAM("control intervals:\n" << control_intervals.transpose());
  ROS_INFO_STREAM("prediction_time:\n" << prediction_time.transpose());
  
  Eigen::MatrixXd free_resp_mtx;
  Eigen::MatrixXd forced_resp_mtx;
  Eigen::MatrixXd position_free_resp_mtx;
  Eigen::MatrixXd position_forced_resp_mtx;
  Eigen::MatrixXd velocity_free_resp_mtx;
  Eigen::MatrixXd velocity_forced_resp_mtx;
  
  if (!thor::math::computeEvolutionMatrix(prediction_time,control_intervals,nax,free_resp_mtx,forced_resp_mtx))
    return -1;
  
  thor::math::splitResponses(free_resp_mtx,velocity_free_resp_mtx,position_free_resp_mtx,forced_resp_mtx,velocity_forced_resp_mtx,position_forced_resp_mtx,nax);
  
  Eigen::VectorXd x0(2*nax);
  x0.setZero();
  x0(3,0)=-0.1;
  
  Eigen::VectorXd u(nax*control_intervals.size(),1);
  u.setZero();
//   u(0,0)=1;
  u(3,0)=2;
  
  ROS_INFO_STREAM("Free Evolution matrix:\n" << free_resp_mtx);
  ROS_INFO_STREAM("VEL Free Evolution matrix:\n" << velocity_free_resp_mtx);
  ROS_INFO_STREAM("POS Free Evolution matrix:\n" << position_free_resp_mtx);
  ROS_INFO_STREAM("Forced Evolution  matrix:\n" << forced_resp_mtx);
  ROS_INFO_STREAM("VEL Forced Evolution  matrix:\n" << velocity_forced_resp_mtx);
  ROS_INFO_STREAM("POS Forced Evolution  matrix:\n" << position_forced_resp_mtx);
  
  
  
  ROS_INFO_STREAM("Free*x0+Forced*u:\n" << free_resp_mtx*x0+forced_resp_mtx*u);
  
  
  return 0;  
}