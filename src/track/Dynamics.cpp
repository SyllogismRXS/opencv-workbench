#include <iostream>

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

#include "Dynamics.h"

using std::cout;
using std::endl;

using namespace boost::numeric::odeint;

Dynamics::Dynamics() : PI(3.14159265359), nd_(0, 1.0), var_nor(rng_, nd_),
                       process_noise_(0), measurement_noise_(0)
{
     for (int i = 0; i < 5; i++) {
          u_[i] = 0;
          x0_[i] = 0;
     }
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
     }
}

void Dynamics::compute_trajectory()
{
     headings_.clear();
     truth_points_.clear();

     if (model_ == cart || model_ == roomba) { 
          state_3d_type x = {x0_[0], x0_[1], x0_[2]};     
          runge_kutta4< state_3d_type > stepper;     
          
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
               state_.push_back(state);
          
               if (model_ == cart) {
                    stepper.do_step(make_ode_wrapper( *this , &Dynamics::cart_model ), x , *it , dt_ );
               } else { 
                    stepper.do_step(make_ode_wrapper( *this , &Dynamics::roomba_model ), x , *it , dt_ );
               }
          } 
     } else if (model_ == constant_velocity) {
     }
}

void Dynamics::set_input(state_5d_type input)
{     
     for(int i = 0; i < 5; i++) {
          u_[i] = input[i];          
     }
}

void Dynamics::cart_model(const state_3d_type &x , state_3d_type &dxdt , double t)
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
}

void Dynamics::roomba_model(const state_3d_type &x , state_3d_type &dxdt , double t)
{
     /// 0 : x-position
     /// 1 : y-position
     /// 2 : theta

     double R = 1;
     double b = 0.5;
	  
     double w_r = u_[0];
     double w_l = u_[1];
     
     dxdt[0] = R*cos(x[2])*(w_r + w_l)/2;
     dxdt[1] = R*sin(x[2])*(w_r + w_l)/2;
     dxdt[2] = R*(w_r-w_l)/(2*b);
}
