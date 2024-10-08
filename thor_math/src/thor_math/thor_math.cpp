#include <thor_math/thor_math.h>

namespace thor 
{
namespace math
{

Eigen::MatrixXd freeResponse ( const double& t, const unsigned int& nax )
{
  Eigen::MatrixXd mtx(2*nax,2*nax);
  mtx.setIdentity();
  mtx.block(0,nax,nax,nax)=Eigen::MatrixXd::Identity(nax,nax)*t;
  return mtx;
}
  
Eigen::MatrixXd forcedResponse ( const double& t, const unsigned int& nax )
{
  Eigen::MatrixXd mtx(2*nax,nax);
  mtx.block(0,   0,nax,nax)=std::pow(t,2)*0.5*Eigen::MatrixXd::Identity(nax,nax);
  mtx.block(nax, 0,nax,nax)=Eigen::MatrixXd::Identity(nax,nax)*t;
  return mtx;
}

  
bool computeEvolutionMatrix ( const Eigen::Ref< Eigen::VectorXd > prediction_time, const Eigen::Ref< Eigen::VectorXd > control_intervals, const unsigned int& nax, Eigen::MatrixXd& free_response, Eigen::MatrixXd& forced_response )
{
  unsigned int np=prediction_time.size();
  if (np==0)
    return false;
  
  unsigned int nc=control_intervals.size();
  if (nc==0)
    return false;
  
  Eigen::VectorXd stop_control_time(nc);
  Eigen::VectorXd start_control_time(nc);
  start_control_time(0)=0;
  stop_control_time(0)=control_intervals(0);
  for (unsigned int ic=1;ic<nc;ic++)
  {
    start_control_time(ic) = stop_control_time(ic-1);
    stop_control_time(ic)  = stop_control_time(ic-1)  + control_intervals(ic);
  }
  free_response.resize(2*nax*np,2*nax);
  forced_response.resize(2*nax*np,nc*nax);
  free_response.setZero();
  forced_response.setZero();
  for (unsigned int ip=0;ip<np;ip++)
  {
    free_response.block(ip*2*nax, 0,   2*nax, 2*nax) = freeResponse(prediction_time(ip),nax);
    for (unsigned int ic=0;ic<nc;ic++)
    {
      if (prediction_time(ip)>stop_control_time(ic))
      {
        double free_response_time=prediction_time(ip)-stop_control_time(ic);
        forced_response.block(ip*2*nax,nax*ic,2*nax,nax)= freeResponse(free_response_time,nax)*forcedResponse(control_intervals(ic),nax);
      }
      else if (prediction_time(ip)>start_control_time(ic))
      {
        forced_response.block(ip*2*nax,nax*ic,2*nax,nax)= forcedResponse(prediction_time(ip)-start_control_time(ic),nax);
      }
      // else zero
    }
  }
  
  return true;
  
}

bool computeJerkEvolutionMatrix ( const Eigen::Ref< Eigen::VectorXd > prediction_time, const Eigen::Ref< Eigen::VectorXd > control_intervals, const unsigned int& nax, Eigen::MatrixXd& free_response, Eigen::MatrixXd& forced_response )
{
  unsigned int np=prediction_time.size();
  if (np==0)
    return false;

  unsigned int nc=control_intervals.size();
  if (nc==0)
    return false;

  forced_response.resize(np*nax,nc*nax);
  free_response.resize(np*nax,nax);

  free_response.setZero();
  free_response.block(0,0,nax,nax)=-Eigen::MatrixXd::Identity(nax,nax)*1/control_intervals(0);

  forced_response.setZero();
  forced_response.block(0,0,nax,nax)=Eigen::MatrixXd::Identity(nax,nax)*1/control_intervals(0);
  for (unsigned int idx=1;idx<nc;idx++)
  {
    forced_response.block(nax*idx,nax*(idx-1),nax,nax)=-Eigen::MatrixXd::Identity(nax,nax)*1/control_intervals(idx);
    forced_response.block(nax*idx,nax*idx,nax,nax)    = Eigen::MatrixXd::Identity(nax,nax)*1/control_intervals(idx);
  }

  return true;

}

bool quadraticControlIntervals ( const double& control_horizon_time, const unsigned int& n_control, const double& first_interval, Eigen::VectorXd& control_intervals, Eigen::VectorXd& prediction_time )
{
  assert(first_interval>0);
  assert(control_horizon_time>0);
  assert(n_control>0);
  
  unsigned int n_steps=control_horizon_time/first_interval;
  
  double a = (n_steps-1)/std::pow(n_control-1,2);
  double b = -2.0*(n_steps-1)/std::pow(n_control-1,2);
  double c = 1+a;
  
  prediction_time.resize(n_control);
  control_intervals.resize(n_control);
  for (unsigned int ic=0;ic<n_control;ic++)
  {
    prediction_time(ic)=first_interval*std::round(a* std::pow(ic+1,2)+b*(ic+1)+c);
    if (ic==0)
      control_intervals(ic)=prediction_time(ic);
    else
      control_intervals(ic)=prediction_time(ic)-prediction_time(ic-1);
  }
  return true;
}

bool constantControlIntervals ( const double& control_horizon_time, const unsigned int& n_control, const double& sampling_period, Eigen::VectorXd& control_intervals, Eigen::VectorXd& prediction_time )
{
  assert(control_horizon_time>0);
  assert(n_control>0);

  double step_size=sampling_period*std::round(control_horizon_time/n_control/sampling_period);

  prediction_time.resize(n_control);
  control_intervals.resize(n_control);
  for (unsigned int ic=0;ic<n_control;ic++)
  {
    prediction_time(ic)=step_size*(ic+1);
    if (ic==0)
      control_intervals(ic)=prediction_time(ic);
    else
      control_intervals(ic)=prediction_time(ic)-prediction_time(ic-1);
  }
  return true;
}

void splitResponses ( const Eigen::MatrixXd& free_response, 
                      Eigen::MatrixXd& velocity_free_response, 
                      Eigen::MatrixXd& position_free_response, 
                      const Eigen::MatrixXd& forced_response, 
                      Eigen::MatrixXd& velocity_forced_response, 
                      Eigen::MatrixXd& position_forced_response, 
                      const unsigned int& nax )
{
  velocity_free_response.resize(free_response.rows()/2,free_response.cols()/2);
  position_free_response.resize(free_response.rows()/2,free_response.cols());
  velocity_forced_response.resize(forced_response.rows()/2,forced_response.cols());
  position_forced_response.resize(forced_response.rows()/2,forced_response.cols());
  
  unsigned int np = free_response.rows()/2/nax;
  for (unsigned int ip=0; ip<np;ip++)
  {
    velocity_free_response.block(   ip*nax,0,nax,free_response.cols()/2)   = free_response.block(  ip*2*nax+nax,nax,nax,free_response.cols()/2);
    position_free_response.block(   ip*nax,0,nax,free_response.cols())   = free_response.block(  ip*2*nax,    0,nax,free_response.cols());
    velocity_forced_response.block( ip*nax,0,nax,forced_response.cols()) = forced_response.block(ip*2*nax+nax,0,nax,forced_response.cols());
    position_forced_response.block( ip*nax,0,nax,forced_response.cols()) = forced_response.block(ip*2*nax,    0,nax,forced_response.cols());
  }
  
}




ThorQP::ThorQP()
{
  m_are_matrices_updated=false;
  m_are_position_bounds_active=false;
  m_are_torque_bounds_active=false;
}

void ThorQP::setConstraints ( const Eigen::VectorXd& qmax, const Eigen::VectorXd& qmin, const Eigen::VectorXd& Dqmax, const Eigen::VectorXd& DDqmax, const Eigen::VectorXd& tau_max )
{
  m_qmax=qmax;
  m_qmin=qmin;
  m_Dqmax=Dqmax;
  m_DDqmax=DDqmax;
  m_tau_max=tau_max;
  m_are_matrices_updated=false;
  
}

void ThorQP::activatePositionBounds(const bool enable_pos_bounds)
{
  if (m_are_position_bounds_active!=enable_pos_bounds)
  {
    m_are_position_bounds_active=enable_pos_bounds;
    m_are_matrices_updated=false;
    ROS_INFO("Position bounds activated. Execute update matrices to load the new options.");
  }
}

void ThorQP::activateTorqueBounds(const bool enable_tau_bounds)
{
  if (m_are_position_bounds_active!=enable_tau_bounds)
  {
    m_are_torque_bounds_active=enable_tau_bounds;
    m_are_matrices_updated=false;
    ROS_INFO("Torque bounds activated. Execute update matrices to load the new options.");
  }
}

bool ThorQP::arePositionBoundsActive()
{
  return m_are_position_bounds_active;
}

void ThorQP::setIntervals ( const unsigned int& num_of_intervals, 
                            const unsigned int& num_of_joints,
                            const double& control_horizon_time, 
                            const double& computing_period )
{
  m_dt=computing_period;
  m_control_horizon_time=control_horizon_time;
  m_nax=num_of_joints;
  m_nc=num_of_intervals;
  
  m_are_matrices_updated=false;

  m_use_input_blocking=true;
  
  m_sol.resize( (m_nax+1)*m_nc);
  m_sol.setZero();
  
}

void ThorQP::setIntervals ( const unsigned int& num_of_intervals,
                            const unsigned int& num_of_joints,
                            const double& control_horizon_time,
                            const double& computing_period,
                            const bool & use_input_blocking)
{
  m_dt=computing_period;
  m_control_horizon_time=control_horizon_time;
  m_nax=num_of_joints;
  m_nc=num_of_intervals;

  m_are_matrices_updated=false;

  if (use_input_blocking)
    m_use_input_blocking=true;
  else
    m_use_input_blocking=false;

  m_sol.resize( (m_nax+1)*m_nc);
  m_sol.setZero();

}

void ThorQP::setWeigthFunction ( const double& lambda_acc, const double& lambda_tau, const double& lambda_jerk, const double& lambda_scaling, const double& lambda_clik )
{
  m_lambda_acc=lambda_acc;
  m_lambda_tau=lambda_tau;
  m_lambda_jerk=lambda_jerk;
  m_lambda_scaling=lambda_scaling;
  m_lambda_clik=lambda_clik;
  m_are_matrices_updated=false;
}

void ThorQP::updateMatrices()
{
  m_CE.resize((m_nax+1)*m_nc,0);
  m_ce0.resize(0);
  if (m_use_input_blocking)
    quadraticControlIntervals(m_control_horizon_time,m_nc,m_dt,m_control_intervals,m_prediction_time);
  else
    constantControlIntervals(m_control_horizon_time,m_nc,m_dt,m_control_intervals,m_prediction_time);
  computeEvolutionMatrix(m_prediction_time,m_control_intervals,m_nax,m_free_response,m_forced_response);
  computeJerkEvolutionMatrix(m_prediction_time,m_control_intervals,m_nax,m_jerk_free_response,m_jerk_forced_response);
  thor::math::splitResponses(m_free_response,m_velocity_free_resp,m_position_free_resp,m_forced_response,m_velocity_forced_resp,m_position_forced_resp,m_nax);
  m_lb.resize(m_nc*(m_nax+1));
  m_ub.resize(m_nc*(m_nax+1));
  for (unsigned int ic=0;ic<m_nc;ic++)
  {
    m_lb.segment(ic*m_nax,m_nax) = -m_DDqmax;
    m_ub.segment(ic*m_nax,m_nax) =  m_DDqmax;
  }
  m_lb.tail(m_nc).setConstant(0.05);
  m_ub.tail(m_nc).setConstant(1.01);

  /*
   * I u > lb    ->  I*u+lb>0
   * I u < ub    -> -I*u+ub>0
   * Fv u +fv*v0 > -Dqmax    ->   Fv*u+fv*v0+Dqmax>0
   * Fv u +fv*v0 <  Dqmax    ->  -Fv*u-fv*v0+Dqmax>0
   * 
   * A^T =[I            -I          Fv^T    -Fv^T ]
   *       nc*(nax+1)   nc*(nax+1)  nc*nax  nc*nax
   * 
   * 
   */
  m_CI.resize(m_nc*(m_nax+1),  4*m_nc*m_nax+2*m_nc);
  m_CI.setZero();
  m_CI.block(0,0,m_nc*(m_nax+1),m_nc*(m_nax+1)).setIdentity();
  m_CI.block(0,m_nc*(m_nax+1),m_nc*(m_nax+1),m_nc*(m_nax+1))=-m_CI.block(0,0,m_nc*(m_nax+1),m_nc*(m_nax+1));
  m_CI.block(0,2*m_nc*(m_nax+1),           m_nc*m_nax,m_nc*m_nax)=m_velocity_forced_resp.transpose();
  m_CI.block(0,2*m_nc*(m_nax+1)+m_nc*m_nax,m_nc*m_nax,m_nc*m_nax)=-m_velocity_forced_resp.transpose();
  m_ci0.resize(m_CI.cols());
  m_ci0.setZero();
  m_ci0.head(m_nc*(m_nax+1))=-m_lb; 
  m_ci0.segment(m_nc*(m_nax+1),m_nc*(m_nax+1))=m_ub;
  for (unsigned int ic=0;ic<m_nc;ic++)
  {
    m_ci0.segment(2*m_nc*(m_nax+1)           +ic*m_nax,m_nax)=m_Dqmax;  // A^T*u>-Dqmax -> A^T*u+Dqmax>0
    m_ci0.segment(2*m_nc*(m_nax+1)+m_nax*m_nc+ic*m_nax,m_nax)=m_Dqmax; // A^T*u<Dqmax ->  A^T*u-Dqmax<0 ->  -A^T*u+Dqmax>0
  }

  /*
   * POSITION BOUNDS
   *
   * Fp u + fp*[p0;v0] > qmin ->  Fp*u+fp*[p0;v0]-qmin>0
   * Fp u + fp*[p0;v0] < qmax -> -Fp*u-fp*[p0;q0]+qmax>0
   *
   * INVARIANCE CONSTRAINTS
   *
   * K=0.99*DDqmax/Dqmax
   *  (K*Fp+Fv)*u+(K*fp+fv)*[p0;v0]-K*qmin>0
   * -(K*Fp+Fv)*u-(K*fp+fv)*[p0;v0]+K*qmax>0
   *
   *
   * A^T = [ A^T   Fp^T   -Fp^T   (K*Fp+Fv)^T   -(K*Fp+Fv)^T ]
   *              nc*nax  nc*nax    nc*nax          nc*nax
   *
   */

  if (m_are_position_bounds_active)
  {
    m_CI.conservativeResize(m_nc*(m_nax+1),  8*m_nc*m_nax+2*m_nc);
    m_CI.block(0,4*m_nc*m_nax+2*m_nc,m_nc*(m_nax+1),4*m_nc*m_nax).setZero();

    m_CI.block(0,4*m_nc*m_nax+2*m_nc,m_nc*m_nax,m_nc*m_nax)=m_position_forced_resp.transpose();
    m_CI.block(0,5*m_nc*m_nax+2*m_nc,m_nc*m_nax,m_nc*m_nax)=-m_position_forced_resp.transpose();

    m_ci0.conservativeResize(8*m_nc*m_nax+2*m_nc);
    m_ci0.tail(4*m_nc*m_nax).setZero();
    for (unsigned int ic=0;ic<m_nc;ic++)
    {
      m_ci0.segment(4*m_nc*m_nax+2*m_nc+ic*m_nax,m_nax)=-m_qmin;
      m_ci0.segment(5*m_nc*m_nax+2*m_nc+ic*m_nax,m_nax)= m_qmax;
    }

    Eigen::MatrixXd Kinv(m_nc*m_nax,m_nc*m_nax);
    Kinv.setZero();
    Eigen::MatrixXd Kinv_block=0.99*(m_DDqmax.cwiseQuotient(m_Dqmax)).asDiagonal();
    for (unsigned int ic=0;ic<m_nc;ic++)
    {
      Kinv.block(ic*m_nax,ic*m_nax,m_nax,m_nax)=Kinv_block;
      m_ci0.segment(6*m_nc*m_nax+2*m_nc+ic*m_nax,m_nax)=-Kinv_block*m_qmin;
      m_ci0.segment(7*m_nc*m_nax+2*m_nc+ic*m_nax,m_nax)= Kinv_block*m_qmax;
    }
    m_CI.block(0,6*m_nc*m_nax+2*m_nc,m_nc*m_nax,m_nc*m_nax)=(Kinv*m_position_forced_resp+m_velocity_forced_resp).transpose();
    m_CI.block(0,7*m_nc*m_nax+2*m_nc,m_nc*m_nax,m_nc*m_nax)=-(Kinv*m_position_forced_resp+m_velocity_forced_resp).transpose();

    m_invariance_free_resp=Kinv*m_position_free_resp;

    m_invariance_free_resp.rightCols(m_nax)+=m_velocity_free_resp;
  }

  /*
   * H u + b > -tau_max    ->  H*u+b+tau_max>0
   * H u + b <  tau_max    -> -H*u-b+tau_max>0
   * A^T =[H            -H          ]
   *       nc*(nax+1)   nc*(nax+1)
   *
   *
   */
  if (m_are_torque_bounds_active)
  {
    m_CI.conservativeResize(m_nc*(m_nax+1),  10*m_nc*m_nax+2*m_nc);
    m_CI.block(0,8*m_nc*m_nax+2*m_nc,m_nc*(m_nax+1),2*m_nc*m_nax).setZero();

//    m_CI.block(0,8*m_nc*m_nax+2*m_nc,m_nc*m_nax,m_nc*m_nax)=m_position_forced_resp.transpose();
//    m_CI.block(0,9*m_nc*m_nax+2*m_nc,m_nc*m_nax,m_nc*m_nax)=-m_position_forced_resp.transpose();

    m_ci0.conservativeResize(10*m_nc*m_nax+2*m_nc);
    m_ci0.tail(2*m_nc*m_nax).setZero();
    for (unsigned int ic=0;ic<m_nc;ic++)
    {
      m_ci0.segment(8*m_nc*m_nax+2*m_nc+ic*m_nax,m_nax)= m_tau_max;
      m_ci0.segment(9*m_nc*m_nax+2*m_nc+ic*m_nax,m_nax)= m_tau_max;
    }
  }

  m_next_position_forced_resp=m_position_forced_resp.topRows(m_nax);
  m_next_position_free_resp=m_position_free_resp.topRows(m_nax);
  
  // forced'*free    to be multiplied by v0
  m_f_vel.resize((m_nax+1)*m_nc,m_nax);
  m_f_vel.setZero();
  m_f_vel.topRows(m_nc*m_nax)=m_velocity_forced_resp.transpose()*m_velocity_free_resp;
  
  // m_lambda_clik* forced'*free  to be multiplied by x0 (works only on the first step)
  m_f_pos.resize((m_nax+1)*m_nc,2*m_nax);
  m_f_pos.setZero();
  m_f_pos.topRows(m_nc*m_nax)=m_lambda_clik*m_next_position_forced_resp.transpose()*m_next_position_free_resp;

  // m_lambda_scaling* ones to be multiplied by sref
  m_f_scaling.resize((m_nax+1)*m_nc);
  m_f_scaling.setZero();
  m_f_scaling.tail(m_nc).setConstant(-m_lambda_scaling);
  
  m_do_scaling.resize(m_nax*m_nc,m_nc);
  m_do_scaling.setZero();
  for (unsigned int ic=0;ic<m_nc;ic++)
    m_do_scaling.block(ic*m_nax,ic,m_nax,1).setOnes();
  
  m_H_fixed.resize((m_nax+1)*m_nc,(m_nax+1)*m_nc);
  m_H_fixed.setZero();

  m_H_variable.resize((m_nax+1)*m_nc,(m_nax+1)*m_nc);
  m_H_variable.setZero();
  
  m_H_fixed.block(0,0,m_nax*m_nc,m_nax*m_nc) =  m_velocity_forced_resp.transpose()*m_velocity_forced_resp+
                                                m_lambda_acc*Eigen::MatrixXd::Identity(m_nax*m_nc,m_nax*m_nc) +
                                                m_lambda_clik*m_next_position_forced_resp.transpose()*m_next_position_forced_resp;
  m_H_fixed.block(m_nax*m_nc,m_nax*m_nc,m_nc,m_nc)  =Eigen::MatrixXd::Identity(m_nc,m_nc)*m_lambda_scaling;
  
  m_H.resize((m_nax+1)*m_nc,(m_nax+1)*m_nc);
  m_f.resize((m_nax+1)*m_nc);
  m_H.setZero();
  m_f.setZero();
  
  m_prediction_pos.resize(m_nax*m_nc);
  m_prediction_vel.resize(m_nax*m_nc);

}

void ThorQP::computeActualMatrices ( const Eigen::VectorXd& targetDq, const Eigen::VectorXd& next_targetQ, const double& target_scaling, const Eigen::VectorXd& x0 )
{
  Eigen::MatrixXd DQT=targetDq.asDiagonal()*m_do_scaling;
  
  m_H_variable.block(0,m_nax*m_nc,m_nax*m_nc,m_nc)=-m_velocity_forced_resp.transpose()*DQT;
  m_H_variable.block(m_nax*m_nc,0,m_nc,m_nax*m_nc)=m_H_variable.block(0,m_nax*m_nc,m_nax*m_nc,m_nc).transpose();
  m_H_variable.block(m_nax*m_nc,m_nax*m_nc,m_nc,m_nc)=DQT.transpose()*DQT;
  
  m_f = m_f_vel*x0.tail(m_nax)+m_f_pos*x0+m_f_scaling*target_scaling;
  m_f.head(m_nc*m_nax) -= m_lambda_clik* (m_next_position_forced_resp.transpose()*next_targetQ).col(0);
  
  m_f.tail(m_nc) -= DQT.transpose()*m_velocity_free_resp*x0.tail(m_nax);

  if (0)
  {
    for (unsigned int ic=0; ic<m_nc; ic++)
    {
      Eigen::VectorXd qc  = m_prediction_pos.block(ic*m_nax,0,m_nax,1);
      Eigen::VectorXd Dqc = m_prediction_vel.block(ic*m_nax,0,m_nax,1);
      
      Eigen::VectorXd non_linear_part_torque=m_chain->getJointTorqueNonLinearPart(qc,Dqc);
      Eigen::MatrixXd inertia_matrix = m_chain->getJointInertia(qc);
      
      m_H_variable.block(ic*m_nax,ic*m_nax,m_nax,m_nax) += m_lambda_tau * inertia_matrix.transpose()*inertia_matrix;
      m_f.block(ic*m_nax,0,m_nax,1)                     += m_lambda_tau * non_linear_part_torque.transpose()*inertia_matrix;
    }
  }

  m_H_variable.block(0,0,m_nax*m_nc,m_nax*m_nc) += m_lambda_jerk * m_jerk_forced_response.transpose()*m_jerk_forced_response;
  m_f.segment(0,m_nax*m_nc)                     += m_lambda_jerk * ((m_jerk_free_response*(m_sol.head(m_nax))).transpose()*m_jerk_forced_response);

  m_H=m_H_fixed+m_H_variable;
  
}

void ThorQP::setInitialState ( const Eigen::VectorXd& x0 )
{
  assert(x0.size()==m_nax*2);
  m_state=x0;
  
  for (unsigned int ic=0; ic<m_nc; ic++)
  {
    m_prediction_pos.block(ic*m_nax,0,m_nax,1)=x0.head(m_nax);
    m_prediction_vel.block(ic*m_nax,0,m_nax,1).setZero();
  }
  
}

void ThorQP::updateState ( const Eigen::VectorXd& next_acc )
{
  m_state.head(m_nax)+=(m_state.tail(m_nax)+0.5*next_acc*m_dt)*m_dt;
  m_state.tail(m_nax)+=next_acc*m_dt;
}

Eigen::VectorXd ThorQP::getState()
{
  return m_state;
}

bool ThorQP::computedCostrainedSolution ( const Eigen::VectorXd& targetDq,
                                          const Eigen::VectorXd& next_targetQ, 
                                          const double& target_scaling, 
                                          const Eigen::VectorXd& x0, 
                                          Eigen::VectorXd& next_acc, 
                                          double& next_scaling )
{
  computeActualMatrices(targetDq,next_targetQ,target_scaling,x0);
  Eigen::VectorXd ci0=m_ci0;
  ci0.segment(2*m_nc*(m_nax+1)           ,m_nax*m_nc)+=m_velocity_free_resp*x0.tail(m_nax); // vel lower bounds
  ci0.segment(2*m_nc*(m_nax+1)+m_nax*m_nc,m_nax*m_nc)-=m_velocity_free_resp*x0.tail(m_nax); // vel upper bounds

  if (m_are_position_bounds_active)
  {
    ci0.segment(2*m_nc*(m_nax+1)+2*m_nax*m_nc,m_nax*m_nc)+=m_position_free_resp*x0; // pos lower bounds
    ci0.segment(2*m_nc*(m_nax+1)+3*m_nax*m_nc,m_nax*m_nc)-=m_position_free_resp*x0; // pos upper bounds
    ci0.segment(2*m_nc*(m_nax+1)+4*m_nax*m_nc,m_nax*m_nc)+=m_invariance_free_resp*x0; // invariance lower constraint
    ci0.segment(2*m_nc*(m_nax+1)+5*m_nax*m_nc,m_nax*m_nc)-=m_invariance_free_resp*x0; // invariance upper constraint
  }
  if  (m_are_torque_bounds_active)
  {
    for (unsigned int idx=0;idx<m_nc;idx++)
    {
      m_CI.block(idx*m_nc,8*m_nc*m_nax+2*m_nc,m_nax,m_nax)=m_chain->getJointInertia(m_prediction_pos.segment(idx*m_nax,m_nax)); // update torque constraints
      Eigen::VectorXd torque_nonlinear_part=m_chain->getJointTorqueNonLinearPart(m_prediction_pos.segment(idx*m_nax,m_nax),m_prediction_vel.segment(idx*m_nax,m_nax));
      ci0.segment(2*m_nc*(m_nax+1)+6*m_nax*m_nc+idx*m_nc,m_nax)+=torque_nonlinear_part; // torque lower bounds
      ci0.segment(2*m_nc*(m_nax+1)+7*m_nax*m_nc+idx*m_nc,m_nax)-=torque_nonlinear_part; // torque upper bounds
    }
  }
  Eigen::solve_quadprog(m_H,m_f,m_CE,m_ce0,m_CI,ci0,m_sol );
  next_acc=m_sol.head(m_nax);
  next_scaling=m_sol (m_nax*m_nc);
  m_prediction_vel = m_velocity_forced_resp*m_sol.head(m_nc*m_nax)+m_velocity_free_resp*x0.tail(m_nax);
  m_prediction_pos = m_position_forced_resp*m_sol.head(m_nc*m_nax)+m_position_free_resp*x0;
  
  return true;
}

bool ThorQP::computedUncostrainedSolution ( const Eigen::VectorXd& targetDq,
                                            const Eigen::VectorXd& next_targetQ,
                                            const double& target_scaling,
                                            const Eigen::VectorXd& x0,
                                            Eigen::VectorXd& next_acc,
                                            double& next_scaling )
{
  computeActualMatrices(targetDq,next_targetQ,target_scaling,x0);
  
  m_svd.compute( m_H, Eigen::ComputeThinU | Eigen::ComputeThinV );
  m_sol=-m_svd.solve(m_f);
  next_acc=m_sol.head(m_nax);
  next_scaling=m_sol (m_nax*m_nc);
  return true;
}

void ThorQP::setDynamicsChain(const rosdyn::ChainPtr& chain)
{
  m_chain=chain;
}


}
}
