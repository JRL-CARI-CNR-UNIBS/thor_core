#include <thor_math/thor_math.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <fstream>
// ...


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
void human_circle(double t,  Eigen::Vector3d& pos, double& vel)
{
  double radius = 0.3;               // Smaller, so stays inside workspace
  double omega = 2*M_PI/5.0;
  double center_x = 0.7;             // Further in front of the robot
  double center_y = 0.0;
  double center_z = 0.8;             // Higher up

  pos(0) = radius*cos(omega*t) + center_x;
  pos(1) = radius*sin(omega*t) + center_y;
  pos(2) = center_z;

  vel = -radius*omega*sin(omega*t);
}

 int main(int argc, char **argv){

    using namespace thor::math;

    std::ofstream logfile("/home/galileo/projects/thor_ws/src/thor_core/thor_math/trajectory_log.csv");
    logfile << "time,q1,q2,q3,q4,q5,q6,ph_x,ph_y,ph_z\n";

    std::string urdf_path = "/home/galileo/projects/thor_ws/src/thor_core/thor_math/ur10/ur10.urdf"; // Replace with your URDF path
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);

    ThorQP qp;
    qp.setPinocchioModel(model);
    // Dummy initialization
    unsigned int nax = model.nv; // Number of joints in the model
    unsigned int nc = 5;
    double horizon = 1.0;
    double st = 0.002;
    std::cout << "Setting up ThorQP with " << nax << " joints and " << nc << " intervals." << std::endl;
    
    qp.setIntervals(nc, nax, horizon, st);
    qp.setCBFParameters(2.5,0.15,0.25,1.0);
    qp.setConstraints(Eigen::VectorXd::Constant(nax, 2.0),   // qmax
                        Eigen::VectorXd::Constant(nax, -2.0),  // qmin
                        Eigen::VectorXd::Constant(nax, 1.0),   // Dqmax
                        Eigen::VectorXd::Constant(nax, 5.0),   // DDqmax
                        Eigen::VectorXd::Constant(nax, 10.0)); // tau_max
    std::cout << "Setting weight functions." << std::endl;
    qp.setWeigthFunction(1e-3, 0.0, 1e-6, 1e+2, 1e+4);
    qp.activatePositionBounds(true);
    qp.activateTorqueBounds(false);

    std::cout << "Updating matrices." << std::endl;
        // Set initial state (zero position + velocity)
        Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2 * nax);
        std::cout << "Setting initial state." << std::endl;

        if (qp.needUpdate()) {
            qp.updateMatrices();
            std::cout << "Matrices updated." << std::endl;
        } else {
            std::cout << "No need to update matrices." << std::endl;
        }
        qp.setInitialState(x0);
        std::cout << "setting target  matrices." << std::endl;
        // Define dummy targets
        Eigen::VectorXd targetDq = Eigen::VectorXd::Constant(nax*nc, 0.5);     // target velocities
        Eigen::VectorXd next_targetQ = Eigen::VectorXd::Constant(nax, 0.3); // next target positions
        double target_scaling = 1.0;
        std::cout << "Computing constrained solution." << std::endl;
        // CBF parameters
        double vh = 0.2;
        Eigen::Vector3d p_h(0.4, 0.0, 0.2);
        unsigned int frameId = 3; // assumes a valid frame index in the UR10 model (you may check this)

        // Outputs
        Eigen::VectorXd next_acc;
        double scaling;

        // Run constrained QP solution
        std::cout << "Running computedCostrainedSolution." << std::endl;
        bool success = qp.computedCostrainedSolution(targetDq, next_targetQ, target_scaling, qp.getState(), vh, p_h, frameId, next_acc, scaling);

        // Output results
        std::cout << "Constrained Solution Success: " << (success ? "Yes" : "No") << std::endl;

        if (success) {
            std::cout << "Next Acceleration:\n" << next_acc.transpose() << std::endl;
            std::cout << "Next Scaling: " << scaling << std::endl;
        } else {
            std::cerr << "QP computation failed!" << std::endl;
        }

        Eigen::VectorXd prediction_time = qp.getPredictionTimeInstant();
        int iter = 0;
        double nominal_t = 0.0;

        std::cout << "starting ... " << std::endl;
        while (iter<8000)
        {
            nominal_t+=scaling*st;
            for (unsigned int iax=0;iax<nax;iax++)
            {
            double tmp_pos,tmp_acc;
            triangularWave(nominal_t+prediction_time(0),1,      next_targetQ(iax),targetDq(iax),tmp_acc);
        //      triangularWave(nominal_t+prediction_time(0),scaling,tmsg.position.at(iax),tmsg.velocity.at(iax),tmsg.effort.at(iax));
            
            for (unsigned int ic=1;ic<nc;ic++)
            {
                triangularWave(nominal_t+prediction_time(ic),1,tmp_pos,targetDq(iax+ic*nax),tmp_acc);
            }
            }

            human_circle(nominal_t, p_h, vh);
            std::cout << __LINE__ << " ... " << std::endl;
            printf("aaa\n");
            qp.computedCostrainedSolution(targetDq,next_targetQ,target_scaling,qp.getState(), vh, p_h, frameId,next_acc,scaling);
            std::cout << __LINE__ << " ... " << std::endl;
            qp.updateState(next_acc);
            double t = nominal_t; // your simulation time
            logfile << t;
            for (int i = 0; i < nax; ++i) logfile << "," << qp.getState()[i];   // robot joints
            logfile << "," << p_h(0) << "," << p_h(1) << "," << p_h(2) << "\n"; // human pos
            iter++;

    }
    logfile.close();
    return 0;  
    }





