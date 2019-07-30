#include <thor_math/thor_math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <fstream>
double Ttot=5.0;
Eigen::VectorXd qstart;
Eigen::VectorXd amplitude;

double period=0.5;

void trj( double theta, const unsigned int ndof, Eigen::VectorXd& qdes, Eigen::VectorXd& dqdes )
{
  if (theta>Ttot)
      theta=Ttot;
  double ldm = 6*pow(theta,5)/pow(Ttot,5) - 15*pow(theta,4)/pow(Ttot,4) + 10*pow(theta,3)/pow(Ttot,3);
  double ldm_d = 5*6*pow(theta,4)/pow(Ttot,5) - 4*15*pow(theta,3)/pow(Ttot,4) + 3*10*pow(theta,2)/pow(Ttot,3);

  qdes.resize(ndof);
  dqdes.resize(ndof);
  for (int idx=0;idx<ndof;idx++)
  {
    qdes(idx)=qstart(idx)+amplitude(idx)*sin(2*M_PI*ldm);
    dqdes(idx)=amplitude(idx)*2*M_PI*cos(2*M_PI*ldm)*ldm_d;
  }
}

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

  ROS_INFO("get params");
  std::string base_frame = "ur10_base_link";
  std::string tool_frame = "ur10_forearm_link";
  Ttot=5.0;
  double predictive_horizon=0.1;
  unsigned int number_instants=10;
  int temp_ui;

  if (!nh.getParam("predictive_horizon",predictive_horizon))
  {
    ROS_INFO("predictive_horizon = %f",predictive_horizon);
  }
  if (!nh.getParam("number_instants",temp_ui))
  {
    ROS_INFO("number_instants = %u",number_instants);
  }
  else
    number_instants=temp_ui;
  if (!nh.getParam("total_time",Ttot))
  {
    ROS_INFO("Total time = %f",Ttot);
  }
  if (!nh.getParam("base_frame",base_frame))
  {
  }
  if (!nh.getParam("tool_frame",tool_frame))
  {
  }
  

  double control_horizon=1;
  double st=0.008;
 
  double lambda_acc=1e-40;
  double lambda_tau=0;
  double lambda_scaling=1e-2;//1e-2;
  double lambda_clik=1e0;//1e0;
  
  urdf::Model model;
  model.initParam("robot_description");
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;
  boost::shared_ptr<rosdyn::Chain> chain = rosdyn::createChain(model,base_frame,tool_frame,grav);
  unsigned nax=chain->getActiveJointsNumber();
  Eigen::VectorXd qmax(6);
  Eigen::VectorXd qmin(6);
  Eigen::VectorXd Dqmax(6);
  Eigen::VectorXd DDqmax(6);
  Eigen::VectorXd tau_max(6);

  qmax.setConstant(10);
  qmin.setConstant(-10);
  Dqmax << 2, 2, 3, 3, 3, 3;
  DDqmax << 5, 5, 10, 10, 10, 10;
  tau_max << 200, 200, 100, 50, 50, 50;

  qmax.conservativeResize(nax);
  qmin.conservativeResize(nax);
  Dqmax.conservativeResize(nax);
  DDqmax.conservativeResize(nax);
  tau_max.conservativeResize(nax);

  ROS_INFO_NAMED(nh.getNamespace(),"CREATING THOR");
  thor::math::ThorQP thor;
  thor.setDynamicsChain(chain);
  
  ROS_INFO_NAMED(nh.getNamespace(),"SETTING THOR MATRICES");
  thor.setIntervals(number_instants,nax,control_horizon,st);
  thor.setWeigthFunction(lambda_acc,lambda_tau,0.0,lambda_scaling,lambda_clik);
  thor.setConstraints(qmax,qmin,Dqmax,DDqmax,tau_max);
  thor.activatePositionBounds(true);
  thor.activateTorqueBounds(true);
  if (thor.needUpdate())
  {
    ROS_INFO_NAMED(nh.getNamespace(),"UPDATING THOR MATRICES");
    thor.updateMatrices();
  }
  Eigen::VectorXd prediction_time = thor.getPredictionTimeInstant();

  qstart.resize(6);
  qstart << 0.0, -2.0, 0.0, 0.0, 0.0, 0.5;
  qstart.conservativeResize(nax);
  amplitude.resize(nax);
  amplitude.setConstant(0.5);
  std::vector<double> temp;
  if (nh.getParam("qstart",temp))
  {
    for (unsigned int idx=0;idx<nax;idx++)
      qstart(idx)=temp.at(idx);
  }

  if (nh.getParam("amplitude",temp))
  {
    for (unsigned int idx=0;idx<nax;idx++)
      amplitude(idx)=temp.at(idx);
  }

  Eigen::VectorXd initial_state(2*nax);
  initial_state.setZero();
  initial_state.head(nax)=qstart;
  thor.setInitialState( initial_state );
  Eigen::VectorXd target_Dq(number_instants*nax);
  target_Dq.setZero();
  double target_scaling=1;
  Eigen::VectorXd next_Q(nax);
  next_Q=initial_state.head(nax);
  Eigen::VectorXd next_acc(nax);
  double scaling=1;
  ros::Time init=ros::Time::now();

  ROS_INFO("solving trajectory");
  double current_time=0.0;
  double nominal_t=0;
  double max_computing_time=0.0;
  double mean_scaling=1.0;
  double mean_computing_time=0.0;

  std::stringstream file_name;
  file_name << "./log_thor_Ttot_" << int(Ttot) << "_Np" << int(number_instants) << "_T" << int(predictive_horizon*1000);
  std::ofstream file_log(file_name.str());
  std::ofstream file_tau("./logtau.txt");
  std::ofstream file_q("./logq.txt");
  std::ofstream file_dq("./logdq.txt");
  std::ofstream file_ddq("./logddq.txt");
  
  for (int idx=0;idx<int((Ttot+1)/st);idx++)
  {
    current_time+=st;
    nominal_t+=scaling*st;

    Eigen::VectorXd tmp_pos(nax);
    Eigen::VectorXd tmp_vel(nax);

    trj(nominal_t+prediction_time(0),nax,tmp_pos,tmp_vel);
    next_Q=tmp_pos;
    target_Dq.segment(0,nax)=tmp_vel;
    for (unsigned int ic=1;ic<number_instants;ic++)
    {
      trj(nominal_t+prediction_time(ic),nax,tmp_pos,tmp_vel);
      target_Dq.segment(ic*nax,nax)=tmp_vel;
    }
    ros::Time t0=ros::Time::now();
//     thor.computedUncostrainedSolution(target_Dq,next_Q,target_scaling,thor.getState(),next_acc,scaling);
    thor.computedCostrainedSolution(target_Dq,next_Q,target_scaling,thor.getState(),next_acc,scaling);
    double tcalc=(ros::Time::now()-t0).toSec()*1e3;
    thor.updateState(next_acc);

    Eigen::VectorXd q_measured   =thor.getState().head(nax);
    Eigen::VectorXd dq_measured  =thor.getState().tail(nax);
    Eigen::VectorXd ddq_measured  =next_acc;
    Eigen::VectorXd tau_measured =chain->getJointTorque(q_measured,dq_measured,ddq_measured);

    /* log */
    for (unsigned int idof=0;idof<nax;idof++)
    {
      file_log << q_measured(idof) << '\t';
      file_q << q_measured(idof) << '\t';
      file_dq << dq_measured(idof) << '\t';
      file_ddq << ddq_measured(idof) << '\t';
      file_tau << tau_measured(idof) << '\t';
      if (idof==nax-1)
      {
          file_q   << '\n';
          file_dq  << '\n';
          file_ddq << '\n';
          file_tau << '\n';
      }
    }
    

    if (tcalc>=max_computing_time)
    {
      max_computing_time=tcalc;
      ROS_ERROR_STREAM("max t calc [ms] =" << max_computing_time << ", scaling = " << scaling);
    }
    mean_computing_time=((mean_computing_time)*idx+tcalc)/(idx+1);
    mean_scaling=(mean_scaling*idx+scaling)/(idx+1);
    ROS_INFO("iter=%d",idx);

  }
  ROS_ERROR("max computing time = %f [ms]",max_computing_time);
  ROS_ERROR("mean computing time = %f [ms]",mean_computing_time);
  return 0;  
}
