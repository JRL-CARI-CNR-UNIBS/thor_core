#include <thor_math/thor_math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>




int main(int argc, char **argv){
  ros::init(argc, argv, "test_thor_math");
  ros::NodeHandle nh;
  
  unsigned nax=6;
  unsigned int nc=5;
  double control_horizon=1;
  double st=0.008;
 
  double lambda_acc=1e-4;
  double lambda_tau=0;
  double lambda_scaling=1e-2;//1e-2;
  double lambda_clik=1e3;//1e0;
  
  Eigen::VectorXd qmax(nax);
  Eigen::VectorXd qmin(nax);
  Eigen::VectorXd Dqmax(nax);
  Eigen::VectorXd DDqmax(nax);
  Eigen::VectorXd tau_max(nax);
  
  qmax.setConstant(10);
  qmin.setConstant(-10);
  Dqmax.setConstant(50);
  DDqmax.setConstant(5);
  tau_max.setConstant(10);
  
    
  ROS_INFO_NAMED(nh.getNamespace(),"CREATING THOR");
  thor::math::ThorQP thor;
  
  ROS_INFO_NAMED(nh.getNamespace(),"SETTING THOR MATRICES");
  thor.setIntervals(nc,nax,control_horizon,st);
  thor.setWeigthFunction(lambda_acc,lambda_tau,lambda_scaling,lambda_clik);
  thor.setConstraints(qmax,qmin,Dqmax,DDqmax,tau_max);
  if (thor.needUpdate())
  {
    ROS_INFO_NAMED(nh.getNamespace(),"UPDATING THOR MATRICES");
    thor.updateMatrices();
  }
  Eigen::VectorXd prediction_time = thor.getPredictionTimeInstant();
  Eigen::VectorXd initial_state(2*nax);
  initial_state.setZero();
  initial_state (3)=-1.5708;
  
  thor.setInitialState( initial_state );
  initial_state (1)=-1.5708;
  
  Eigen::VectorXd amplitude(nax);
  amplitude.setZero();
  amplitude(0)=0.5;
  amplitude(1)=-1;
  amplitude(2)=1;
  
  
  Eigen::VectorXd target_Dq(nc*nax);
  target_Dq.setZero();
  
  double target_scaling=1;
  Eigen::VectorXd next_Q(nax);
  next_Q=initial_state.head(nax);
  
  Eigen::VectorXd next_acc(nax);
  double scaling=1;
  
  ros::Time init=ros::Time::now();
  ros::Rate lp(1.0/st);
  
  ros::Publisher pub=nh.advertise<sensor_msgs::JointState>("/joint_pos_target",1);
  ros::Publisher target_pub=nh.advertise<sensor_msgs::JointState>("/joint_target_no_optimal",1);
  ros::Publisher scaling_pub=nh.advertise<std_msgs::Float64>("/scaling",1);
  
  sensor_msgs::JointState msg;
  msg.position.resize(nax);
  msg.velocity.resize(nax);
  msg.effort.resize(nax);
  msg.name.resize(nax);
  
  msg.name.at(0)="shoulder_pan_joint";
  msg.name.at(1)="shoulder_lift_joint";
  msg.name.at(2)="elbow_joint";
  msg.name.at(3)="wrist_1_joint";
  msg.name.at(4)="wrist_2_joint";
  msg.name.at(5)="wrist_3_joint";
  
  sensor_msgs::JointState tmsg;
  tmsg.position.resize(nax);
  tmsg.velocity.resize(nax);
  tmsg.effort.resize(nax);
  tmsg.name=msg.name;
  
  std_msgs::Float64 scaling_msg;
  double nominal_t=0;
  double max_t_calc=0;
  double mean_t_calc=0;
  unsigned int iter=0;
  
  
  while (ros::ok())
  {
    nominal_t+=scaling*st;
    
    
    ros::Time t0=ros::Time::now();
    thor.computedCostrainedSolution(target_Dq,next_Q,target_scaling,thor.getState(),next_acc,scaling);
    double tcalc=(ros::Time::now()-t0).toSec();
    thor.updateState(next_acc);
    
    for (unsigned int iax=0;iax<nax;iax++)
    {
      tmsg.position.at(iax)=next_Q(iax);
      msg.position.at(iax)=thor.getState()(iax);
      msg.velocity.at(iax)=thor.getState()(iax+nax);
      msg.effort.at(iax)=next_acc(iax);
      
    }
    msg.header.stamp=ros::Time::now();
    pub.publish(msg);
    tmsg.header.stamp=ros::Time::now();
    target_pub.publish(tmsg);
    scaling_msg.data=scaling;
    scaling_pub.publish(scaling_msg);
    lp.sleep();
    if (tcalc>max_t_calc)
    {
      max_t_calc=tcalc;
      ROS_ERROR_STREAM("max t calc [ms] =" << tcalc*1000 << ", scaling = " << scaling);
    }
    iter++;
    mean_t_calc=((mean_t_calc)*(iter-1)+tcalc)/iter;
    ROS_WARN_STREAM_THROTTLE(1,"mean t calc [ms] =" << mean_t_calc*1000 << ", max t calc [ms] =" << max_t_calc*1000 << ", scaling = " << scaling);

    
  }
  return 0;  
}