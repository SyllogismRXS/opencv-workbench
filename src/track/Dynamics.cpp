#include <iostream>

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

#include "Dynamics.h"

using std::cout;
using std::endl;

using namespace boost::numeric::odeint;

Dynamics::Dynamics() : PI(3.14159265359), rng_(new boost::mt19937()), 
                       nd_(0, 1.0), var_nor(rng_, nd_),
                       process_noise_(0), measurement_noise_(0)
{
     for (int i = 0; i < 5; i++) {
          u_[i] = 0;
          x0_[i] = 0;
          state_[i] = 0;
     }
     
     rng_->seed((++seed_) + time(NULL));     
}

std::vector<double> Dynamics::time_vector(double t0, double dt, double tend)
{
     std::vector<double> tt;
     for( double t = t0; t < tend ; t += dt ) {
          tt.push_back(t);
     }
     return tt;
}

void Dynamics::set_time(double t0, double dt, double tend)
{
     t0_ = t0;
     dt_ = dt;
     tend_ = tend;

     tt_.clear();
     for( double t = t0_ ; t < tend_ ; t += dt_ ) {
          tt_.push_back(t);
     }
}

void Dynamics::set_x0(state_5d_type x0)
{
     for(int i = 0; i < 5; i++) {
          x0_[i] = x0[i];          
          state_[i] = x0[i];
          measurement_[i] = 0;
     }
     
     measurement_[0] = x0[0] + var_nor() * measurement_noise_;
     measurement_[1] = x0[1] + var_nor() * measurement_noise_;
}

void Dynamics::compute_trajectory()
{
     headings_.clear();
     truth_points_.clear();

     if (model_ == cart || model_ == roomba) { 
          state_5d_type x = {x0_[0], x0_[1], x0_[2], x0_[3], x0_[4]};     
          runge_kutta4< state_5d_type > stepper;     
          
          std::vector<double>::iterator it = tt_.begin();
          for (; it != tt_.end(); it++) {
               cv::Point2d truth, measured;
               
               truth = cv::Point2d(x[0], x[1]);
               
               measured.x = truth.x + var_nor() * measurement_noise_;
               measured.y = truth.y + var_nor() * measurement_noise_;
               
               headings_.push_back(x[2]*180.0/PI);
               truth_points_.push_back(truth);
               measured_points_.push_back(measured);
               
               state_5d_type state;
               state[0] = x[0];
               state[1] = x[1];
               state[2] = x[2];
               state[3] = 0;
               state[4] = 0;
               states_.push_back(state);
          
               if (model_ == cart) {
                    stepper.do_step(make_ode_wrapper( *this , &Dynamics::cart_model ), x , *it , dt_ );
               } else { 
                    stepper.do_step(make_ode_wrapper( *this , &Dynamics::roomba_model ), x , *it , dt_ );
               }
          } 
     } else if (model_ == constant_velocity) {
     }
}

void Dynamics::step_motion_model(double dt, double time)
{
     if (input_sequence_.size() > 0 && input_sequence_.front().time <= time) {
          cout << "Using input: " << endl;
          for (int i = 0; i < 5; i++) {
               u_[i] = input_sequence_.front().input[i];
               
               printf("%d : %f\n", i, u_[i]);
          }
          input_sequence_.erase(input_sequence_.begin());
     }
     
     if (model_ == cart || model_ == roomba) {                          
          if (model_ == cart) {
               stepper_.do_step(make_ode_wrapper( *this , &Dynamics::cart_model ), state_ , 0 , dt );
          } else { 
               stepper_.do_step(make_ode_wrapper( *this , &Dynamics::roomba_model ), state_ , 0 , dt );
          }
     
          cv::Point2d truth, measured;               
          truth = cv::Point2d(state_[0], state_[1]);
               
          measured.x = truth.x + var_nor() * measurement_noise_;
          measured.y = truth.y + var_nor() * measurement_noise_;
               
          measurement_[0] = measured.x;
          measurement_[1] = measured.y;
          measurement_[2] = 0;
          measurement_[3] = 0;
          measurement_[4] = 0;
          
          headings_.push_back(state_[2]*180.0/PI);
          truth_points_.push_back(truth);
          measured_points_.push_back(measured);
     
     } else if (model_ == constant_velocity) {
     }
}

void Dynamics::set_input(state_5d_type input)
{     
     input_sequence_.clear();
     
     ControlInput control_input;
     control_input.time = 0;
     for(int i = 0; i < 5; i++) {
          control_input.input[i] = input[i];
     }          
     
     input_sequence_.push_back(control_input);          
}

void Dynamics::set_input(state_5d_type input, double time)
{
     ControlInput control_input;
     control_input.time = time;
     for(int i = 0; i < 5; i++) {
          control_input.input[i] = input[i];
     }          
     
     input_sequence_.push_back(control_input);          
}

void Dynamics::set_input_sequence(std::vector<ControlInput> &input_sequence)
{
     input_sequence_.clear();
     
     for (std::vector<ControlInput>::iterator it = input_sequence.begin();
          it != input_sequence.end(); it++) {
          
          ControlInput control_input;
          control_input.time = it->time;
          for(int i = 0; i < 5; i++) {
               control_input.input[i] = it->input[i];
          }          
          input_sequence_.push_back(control_input);
     }
}

void Dynamics::cart_model(const state_5d_type &x , state_5d_type &dxdt , double t)
{
     /// 0 : x-position
     /// 1 : y-position
     /// 2 : theta
	  
     //u_[0] = 2;
     //u_[1] = 3.14159265359/20;     
     
     double u = u_[0];
     double u_theta = u_[1];
     double L = 3;
     
     dxdt[0] = u*cos(x[2]) + var_nor() * process_noise_; 
     dxdt[1] = u*sin(x[2]) + var_nor() * process_noise_; 
     dxdt[2] = u/L*tan(u_theta) + var_nor() * process_noise_;
     dxdt[3] = 0;
     dxdt[4] = 0;
}

void Dynamics::roomba_model(const state_5d_type &x , state_5d_type &dxdt , double t)
{
     /// 0 : x-position
     /// 1 : y-position
     /// 2 : theta

     double R = 1;
     double b = 0.5;
	
     double w_l = u_[0];
     double w_r = u_[1];     
     
     dxdt[0] = R*cos(x[2])*(w_r + w_l)/2;
     dxdt[1] = R*sin(x[2])*(w_r + w_l)/2;
     dxdt[2] = R*(w_r-w_l)/(2*b);
     dxdt[3] = 0;
     dxdt[4] = 0;
}


//void Dynamics::aircraft_6DOF_model(const state_type &x , state_type &dxdt , double t)
//{
//     /// 0:  u     : surge velocity
//     /// 1:  v     : sway velocity
//     /// 2:  w     : heave velocity
//     /// 3:  p     : roll rate
//     /// 4:  q     : pitch rate
//     /// 5:  r     : yaw rate
//     /// 6:  xpos  : earth x-pos
//     /// 7:  ypos  : earth y-pos
//     /// 8:  zpos  : earth z-pos
//     /// 9:  phi   : roll angle
//     /// 10: theta : pitch angle
//     /// 11: psi   : yaw angle
//              
//     // Get current state values
//     double u     = x[0];
//     double v     = x[1];
//     double w     = x[2];
//     double p     = x[3];
//     double q     = x[4];
//     double r     = x[5];
//     //double xpos  = x[6];
//     //double ypos  = x[7];
//     //double zpos  = x[8];
//     double phi   = (x[9]);
//     double theta = (x[10]);     
//     double psi   = (x[11]);
//     
//     // Precalculate trig functions
//     double c1 = cos(phi);
//     double c2 = cos(theta); 
//     double c3 = cos(psi); 
//     double s1 = sin(phi); 
//     double s2 = sin(theta); 
//     double s3 = sin(psi); 
//     double t2 = tan(theta);
//
//     //double m = 4536; // mass
//     double m = 1;
//
//     // Thrust is in surge direction
//     
//     // Force Inputs
//     double Fx = u_[0];  
//     double Fy = 0;
//     double Fz = 0;
//     double Fk = u_[1]; // roll
//     double Fm = u_[2]; // pitch
//     double Fn = 0; // yaw     
//
//     // For x-z symmetry:
//     // 1.) Ixy = Iyx = 0
//     // 2.) Iyz = Izy = 0
//     //double Ixx = 23;
//     //double Ixy = 0;
//     //double Ixz = 2.97;
//     //double Iyy = 15.13;
//     //double Iyz = 0;
//     //double Izz = 16.99;
//
//     double Ixx = 1;
//     double Ixy = 0;
//     double Ixz = 0;
//     double Iyy = 1;
//     double Iyz = 0;
//     double Izz = 1;
//
//     //double Ixx = 35926.5;
//     //double Ixy = 0;
//     //double Ixz = 3418.17;
//     //double Iyy = 33940.7;
//     //double Iyz = 0;
//     //double Izz = 67085.5;
//
//     // Calculate Translational Forces
//     double X = Fx - m*q*w + m*r*v;
//     double Y = Fy - m*r*u + m*p*w;
//     double Z = Fz - m*p*v + m*q*u;
//
//     // Calculate Rotational Forces
//     double K = Fk + Iyz*(pow(q,2)-pow(r,2)) + Ixz*p*q - Ixy*r*p + (Iyy-Izz)*q*r;
//     double M = Fm + Ixz*(pow(r,2)-pow(p,2)) + Ixy*q*r - Iyz*p*q + (Izz-Ixx)*r*p;
//     double N = Fn + Ixy*(pow(p,2)-pow(q,2)) + Iyz*r*p - Ixz*q*r + (Ixx-Iyy)*p*q;
//
//     // Calculate angles
//     double P = p + (q*s1 + r*c1)*t2;
//     double Q = q*c1 - r*s1;
//     double R = (q*s1 + r*c1)*1.0/c2;
//     
//     // Calculate Velocities and Rates
//     // xdot(1:6) = inv(A)*[X Y Z K M N]'
//     Eigen::MatrixXd A, Forces, derivs;
//     A.resize(9,9); Forces.resize(9,1);
//     
//     A << m, 0, 0,   0,    0,    0,  0, 0, 0,
//          0, m, 0,   0,    0,    0,  0, 0, 0,
//          0, 0, m,   0,    0,    0,  0, 0, 0,
//          0, 0, 0,  Ixx, -Ixy, -Ixz, 0, 0, 0,
//          0, 0, 0, -Ixy,  Iyy, -Iyz, 0, 0, 0,
//          0, 0, 0, -Ixz, -Iyz,  Izz, 0, 0, 0,
//          0, 0, 0,   0,    0,    0,  1, 0, 0,
//          0, 0, 0,   0,    0,    0,  0, 1, 0,
//          0, 0, 0,   0,    0,    0,  0, 0, 1;
//          
//
//     Forces << X, Y, Z, K, M, N, P, Q, R;
//
//     derivs = A.inverse() * Forces;         
//     
//     dxdt[0] = derivs(0,0);
//     dxdt[1] = derivs(1,0);
//     dxdt[2] = derivs(2,0);
//     dxdt[3] = derivs(3,0);
//     dxdt[4] = derivs(4,0);
//     dxdt[5] = derivs(5,0);
//     
//     // Calculate Positions
//     dxdt[6] = c3*c2*u + (c3*s2*s1-s3*c1)*v + (s3*s1+c3*c1*s2)*w;
//     dxdt[7] = s3*c2*u + (c1*c3+s1*s2*s3)*v + (c1*s2*s3-c3*s1)*w;
//     dxdt[8] = -s2*u + c2*s1*v + c1*c2*w;
//
//     // Calculate Angles
//     dxdt[9] = derivs(6,0);
//     dxdt[10] = derivs(7,0);
//     dxdt[11] = derivs(8,0);
//}
