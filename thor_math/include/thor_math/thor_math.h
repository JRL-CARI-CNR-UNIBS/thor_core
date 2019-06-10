#ifndef __thor_math__
#define __thor_math__


#include "Eigen/Dense"
#include <ros/console.h>
#include <eigen_matrix_utils/eiquadprog.hpp>
#include <rosdyn_core/primitives.h>

namespace thor 
{
namespace math
{
  
bool computeEvolutionMatrix( const Eigen::Ref<Eigen::VectorXd> prediction_time,
                             const Eigen::Ref<Eigen::VectorXd> control_intervals,
                             const unsigned int& nax,
                             Eigen::MatrixXd& free_response,
                             Eigen::MatrixXd& forced_response
                            );

Eigen::MatrixXd freeResponse(const double& t, const unsigned int& nax);
Eigen::MatrixXd forcedResponse(const double& t, const unsigned int& nax);


void splitResponses( const Eigen::MatrixXd& free_response, 
                     Eigen::MatrixXd& velocity_free_response, 
                     Eigen::MatrixXd& position_free_response, 
                     const Eigen::MatrixXd& forced_response, 
                     Eigen::MatrixXd& velocity_forced_response, 
                     Eigen::MatrixXd& position_forced_response, 
                     const unsigned int& nax);

bool quadraticControlIntervals( const double& control_horizon_time, const unsigned int& n_control, const double& first_interval, Eigen::VectorXd& control_intervals, Eigen::VectorXd& prediction_time );

class ThorQP
{
protected:
  bool m_are_matrices_updated;
  
  Eigen::MatrixXd m_weigth_matrix;
  Eigen::MatrixXd m_H_fixed;
  Eigen::MatrixXd m_H_variable;
  Eigen::VectorXd m_f_scaling;  // m_lambda_scaling* ones to be multiplied by sref
  Eigen::MatrixXd m_f_vel; // forced'*free    to be multiplied by v0
  Eigen::MatrixXd m_f_pos; // m_lambda_clik* forced'*free  to be multiplied by x0 (works only on the first step)
  
  Eigen::MatrixXd m_H;
  Eigen::VectorXd m_f; 
  
  Eigen::VectorXd m_sol;
  
  Eigen::VectorXd m_ub;
  Eigen::VectorXd m_lb;
  Eigen::MatrixXd m_CE;
  Eigen::VectorXd m_ce0;
  
  Eigen::MatrixXd m_CI;
  Eigen::VectorXd m_ci0;
  
  Eigen::VectorXd m_qmax;
  Eigen::VectorXd m_qmin;
  Eigen::VectorXd m_Dqmax;
  Eigen::VectorXd m_DDqmax;
  Eigen::VectorXd m_tau_max;

  bool m_are_position_bounds_active;
  
  Eigen::VectorXd m_prediction_pos;
  Eigen::VectorXd m_prediction_vel;
  
  Eigen::VectorXd m_control_intervals;
  Eigen::VectorXd m_prediction_time;
  Eigen::MatrixXd m_forced_response;
  Eigen::MatrixXd m_free_response;
  Eigen::MatrixXd m_position_free_resp;
  Eigen::MatrixXd m_position_forced_resp;
  Eigen::MatrixXd m_next_position_free_resp;
  Eigen::MatrixXd m_next_position_forced_resp;
  Eigen::MatrixXd m_velocity_free_resp;
  Eigen::MatrixXd m_velocity_forced_resp;
  Eigen::MatrixXd m_do_scaling; // applying the scaling to trajectory
  Eigen::MatrixXd m_invariance_free_resp;

  Eigen::JacobiSVD<Eigen::MatrixXd>  m_svd;
  
  unsigned int m_nc; //number of control and prediction intervals
  unsigned int m_nax; //number of joints
  double m_control_horizon_time;
  double m_dt;
  
  double m_lambda_acc;
  double m_lambda_tau;
  double m_lambda_scaling;
  double m_lambda_clik;
  Eigen::VectorXd m_state;
  
  boost::shared_ptr<rosdyn::Chain>  m_chain;
  
  virtual void computeActualMatrices( const Eigen::VectorXd& targetDq,
                              const Eigen::VectorXd& next_targetQ,
                              const double& target_scaling,
                              const Eigen::VectorXd& x0);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ThorQP();

  void setConstraints(const Eigen::VectorXd& qmax,
                 const Eigen::VectorXd& qmin,
                 const Eigen::VectorXd& Dqmax,
                 const Eigen::VectorXd& DDqmax,
                 const Eigen::VectorXd& tau_max);

  void activatePositionBounds(const bool enable_pos_bounds);

  bool arePositionBoundsActive();
  
  void setIntervals(const unsigned int& num_of_intervals,
                    const unsigned int& num_of_joints,
                    const double& control_horizon_time,
                    const double& computing_period);
  
  void setWeigthFunction( const double& lambda_acc, const double& lambda_tau, const double& lambda_scaling, const double& lambda_clik );
  
  bool needUpdate(){return !m_are_matrices_updated;};
  
  virtual void updateMatrices();
  
  virtual bool computedUncostrainedSolution(  const Eigen::VectorXd& targetDq,
                                      const Eigen::VectorXd& next_targetQ,
                                      const double& target_scaling,
                                      const Eigen::VectorXd& x0,
                                      Eigen::VectorXd& next_acc,
                                      double& next_scaling
                                   );
  virtual bool computedCostrainedSolution(  const Eigen::VectorXd& targetDq,
                                      const Eigen::VectorXd& next_targetQ,
                                      const double& target_scaling,
                                      const Eigen::VectorXd& x0,
                                      Eigen::VectorXd& next_acc,
                                      double& next_scaling
  );
  
  void setInitialState(const Eigen::VectorXd& x0);
  void updateState( const Eigen::VectorXd& next_acc );
  Eigen::VectorXd getState();
  Eigen::VectorXd getPredictionTimeInstant(){return m_prediction_time;};
  
  
  void setDynamicsChain(const boost::shared_ptr<rosdyn::Chain>&  chain);
};

}
}

#endif
