#include <thor_math/thor_math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

void triangularWave(double t, const double& scaling, double& pos, double& vel, double& acc)
{
  double amplitude=2;
  double period=5;
  
  while(t>period)
    t-=period;
  
  acc=0;
  if (t<0.5*period)
  {
    pos=amplitude*t/(0.5*period);
    vel=amplitude/(0.5*period)*scaling;
  }
  else
  {
    pos=amplitude-amplitude*(t-0.5*period)/(0.5*period);
    vel=-amplitude/(0.5*period)*scaling;
  }
  
}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_thor_math");
  ros::NodeHandle nh;
  
  unsigned nax=6;
  unsigned int nc=5;
  double control_horizon=1;
  double st=0.001;
 
  double lambda_acc=1e-6;
  double lambda_tau=0;
  double lambda_scaling=1e-1;
  double lambda_clik=1e-2;
  double lamda_jerk=1e-12;
  
  Eigen::VectorXd qmax(nax);
  Eigen::VectorXd qmin(nax);
  Eigen::VectorXd Dqmax(nax);
  Eigen::VectorXd DDqmax(nax);
  Eigen::VectorXd tau_max(nax);
  
  qmax.setConstant(10);
  qmin.setConstant(-10);
  Dqmax.setConstant(2);
  DDqmax.setConstant(5);
  tau_max.setConstant(100);
  
    
  ROS_INFO_NAMED(nh.getNamespace(),"CREATING THOR");
  thor::math::ThorQP thor;
  
  ROS_INFO_NAMED(nh.getNamespace(),"SETTING THOR MATRICES");
  thor.setIntervals(nc,nax,control_horizon,st);
  thor.setWeigthFunction(lambda_acc,lambda_tau,lamda_jerk,lambda_scaling,lambda_clik);
  thor.setConstraints(qmax,qmin,Dqmax,DDqmax,tau_max);
  thor.activateTorqueBounds(true);
  if (thor.needUpdate())
  {
    ROS_INFO_NAMED(nh.getNamespace(),"UPDATING THOR MATRICES");
    thor.updateMatrices();
  }
  Eigen::VectorXd prediction_time = thor.getPredictionTimeInstant();
  Eigen::VectorXd state(2*nax);
  state.setZero();
  thor.setInitialState(state);
  
  Eigen::VectorXd target_Dq(nc*nax);
  target_Dq.setOnes();
  double target_scaling=1;
  Eigen::VectorXd next_Q(nax);
  next_Q.setZero();
  
  Eigen::VectorXd next_acc(nax);
  double scaling=1;
  
  ros::Time init=ros::Time::now();
  ros::Rate lp(1.0/st);
  
  ros::Publisher pub=nh.advertise<sensor_msgs::JointState>("/joint_states",1);
  ros::Publisher target_pub=nh.advertise<sensor_msgs::JointState>("/joint_target",1);
  ros::Publisher scaling_pub=nh.advertise<std_msgs::Float64>("/scaling",1);
  
  sensor_msgs::JointState msg;
  msg.position.resize(nax);
  msg.velocity.resize(nax);
  msg.effort.resize(nax);
  
  sensor_msgs::JointState tmsg;
  tmsg.position.resize(nax);
  tmsg.velocity.resize(nax);
  tmsg.effort.resize(nax);
  
  std_msgs::Float64 scaling_msg;
  double nominal_t=0;
  double max_t_calc=0;
  double mean_t_calc=0;
  unsigned int iter=0;
  while (ros::ok())
  {
    nominal_t+=scaling*st;
    for (unsigned int iax=0;iax<nax;iax++)
    {
      double tmp_pos,tmp_acc;
      triangularWave(nominal_t+prediction_time(0),1,      next_Q(iax),target_Dq(iax),tmp_acc);
      triangularWave(nominal_t+prediction_time(0),scaling,tmsg.position.at(iax),tmsg.velocity.at(iax),tmsg.effort.at(iax));
      
      for (unsigned int ic=1;ic<nc;ic++)
      {
        triangularWave(nominal_t+prediction_time(ic),1,tmp_pos,target_Dq(iax+ic*nax),tmp_acc);
      }
    }
    
//     ROS_INFO("Step %d",i);
    ros::Time t0=ros::Time::now();
//     thor.computedUncostrainedSolution(target_Dq,next_Q,target_scaling,thor.getState(),next_acc,scaling);
    thor.computedCostrainedSolution(target_Dq,next_Q,target_scaling,thor.getState(),next_acc,scaling);
    double tcalc=(ros::Time::now()-t0).toSec();
    thor.updateState(next_acc);
    
    for (unsigned int iax=0;iax<nax;iax++)
    {
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
    
//     return 0;
  }
  return 0;  
}
