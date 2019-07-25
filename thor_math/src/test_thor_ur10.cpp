#include <thor_math/thor_math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>


double period=0.5;
void triangularWave(double t, const double& scaling, double& pos, double& vel, double& acc, double offset, double amplitude)
{
  
  while(t>period)
    t-=period;
  
  acc=0;
  if (t<=0.5*period)
  {
    double x=t/(0.5*period);
    pos=offset+amplitude*(3*std::pow(x,2)-2*std::pow(x,3));
    vel=amplitude/(0.5*period)*(6*std::pow(x,1)-6*std::pow(x,2))*scaling;
    acc=amplitude/std::pow(0.5*period,2)*(6-12*std::pow(x,1))*scaling*scaling;
  }
  else
  {
    double x=(t-0.5*period)/(0.5*period);
    pos=offset+amplitude-amplitude*(3*std::pow(x,2)-2*std::pow(x,3));
    vel=-amplitude/(0.5*period)*(6*std::pow(x,1)-6*std::pow(x,2))*scaling;
    acc=-amplitude/std::pow(0.5*period,2)*(6-12*std::pow(x,1))*scaling*scaling;
  }
  
}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_thor_math");
  ros::NodeHandle nh;
  
  unsigned nax=7;
  unsigned int nc=5;
  double control_horizon=1;
  double st=0.008;
 
  double lambda_acc=1e-40;
  double lambda_tau=0;
  double lambda_scaling=1e-2;//1e-2;
  double lambda_clik=1e0;//1e0;
  
  Eigen::VectorXd qmax(nax);
  Eigen::VectorXd qmin(nax);
  Eigen::VectorXd Dqmax(nax);
  Eigen::VectorXd DDqmax(nax);
  Eigen::VectorXd tau_max(nax);
  
  qmax.setConstant(10);
  qmin.setConstant(-10);
  Dqmax.setConstant(1);
  DDqmax.setConstant(3);
  tau_max.setConstant(100);

  urdf::Model model;
  model.initParam("robot_description");
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;
  std::string base_frame = "world";
  std::string tool_frame = "ur5_32_ee_link";
  boost::shared_ptr<rosdyn::Chain> chain = rosdyn::createChain(model,base_frame,tool_frame,grav);
    
  ROS_INFO_NAMED(nh.getNamespace(),"CREATING THOR");
  thor::math::ThorQP thor;
  thor.setDynamicsChain(chain);
  
  ROS_INFO_NAMED(nh.getNamespace(),"SETTING THOR MATRICES");
  thor.setIntervals(nc,nax,control_horizon,st);
  thor.setWeigthFunction(lambda_acc,lambda_tau,0.0,lambda_scaling,lambda_clik);
  thor.setConstraints(qmax,qmin,Dqmax,DDqmax,tau_max);
  thor.activateTorqueBounds(true);
  if (thor.needUpdate())
  {
    ROS_INFO_NAMED(nh.getNamespace(),"UPDATING THOR MATRICES");
    thor.updateMatrices();
  }
  Eigen::VectorXd prediction_time = thor.getPredictionTimeInstant();
  Eigen::VectorXd initial_state(2*nax);
  initial_state.setZero();
  initial_state (2)=-1.5708;
  initial_state (4)=-1.5708;
  
  thor.setInitialState( initial_state );
  
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
  msg.name.at(0)="linear_motor_cursor_joint";
  msg.name.at(1)="ur5_shoulder_pan_joint";
  msg.name.at(2)="ur5_shoulder_lift_joint";
  msg.name.at(3)="ur5_elbow_joint";
  msg.name.at(4)="ur5_wrist_1_joint";
  msg.name.at(5)="ur5_wrist_2_joint";
  msg.name.at(6)="ur5_wrist_3_joint";
  
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
    
    for (unsigned int iax=0;iax<nax;iax++)
    {
      double tmp_pos,tmp_acc;
      triangularWave(nominal_t+prediction_time(0),1, next_Q(iax),target_Dq(iax),tmp_acc,initial_state (iax),amplitude(iax));
      triangularWave(nominal_t+prediction_time(0),scaling,tmsg.position.at(iax),tmsg.velocity.at(iax),tmsg.effort.at(iax),initial_state (iax),amplitude(iax));
      
      for (unsigned int ic=1;ic<nc;ic++)
      {
        triangularWave(nominal_t+prediction_time(ic),1,tmp_pos,target_Dq(iax+ic*nax),tmp_acc,initial_state (iax),amplitude(iax));
      }
    }
    if (nominal_t>=3*period)
    {
      nominal_t=3*period;
      target_Dq.setZero();
    }
    
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

    
  }
  return 0;  
}
