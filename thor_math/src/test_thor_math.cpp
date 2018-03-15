#include <thor_math/thor_math.h>
#include <ros/ros.h>



int main(int argc, char **argv){
  ros::init(argc, argv, "test_thor_math");
  ros::NodeHandle nh;
  
  unsigned nax=2;
  unsigned int nc=4;
  double control_horizon=0.1;
  double st=0.01;
 
  double lambda_acc=1e-13;
  double lambda_tau=10;
  double lambda_scaling=1e-8;
  double lambda_clik=0;
  
  
  ROS_INFO_NAMED(nh.getNamespace(),"CREATING THOR");
  thor::math::ThorQP thor;
  
  ROS_INFO_NAMED(nh.getNamespace(),"SETTING THOR MATRICES");
  thor.setIntervals(nc,nax,control_horizon,st);
  thor.setWeigthFunction(lambda_acc,lambda_tau,lambda_scaling,lambda_clik);
  
  if (thor.needUpdate())
  {
    ROS_INFO_NAMED(nh.getNamespace(),"UPDATING THOR MATRICES");
    thor.updateMatrices();
  }
  Eigen::VectorXd state(2*nax);
  state.setZero();
  thor.setInitialState(state);
  
  Eigen::VectorXd target_Dq(nc*nax);
  target_Dq.setOnes();
  double target_scaling=1;
  Eigen::VectorXd next_Q(nax);
  next_Q.setZero();
  
  Eigen::VectorXd next_acc(nax);
  double scaling;
  
  for (int i=0;i<20;i++)
  {
    if (i==10)
      target_Dq.setZero();
    
    next_Q+=target_Dq.head(nax)*st;
    
//     ROS_INFO("Step %d",i);
    thor.computedUncostrainedSolution(target_Dq,next_Q,target_scaling,thor.getState(),next_acc,scaling);
    thor.updateState(next_acc);
    ROS_WARN_STREAM("ITER: "<< i << "\nPOS: " << thor.getState().head(nax).transpose() << "\nVEL: " << thor.getState().tail(nax).transpose() << "\nACC: " << next_acc.transpose());
  }
  return 0;  
}