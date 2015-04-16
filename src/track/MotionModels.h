#ifndef MOTIONMODELS_H_
#define MOTIONMODELS_H_
/// ----------------------------------------------------------------------------
/// @file MotionModels.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2013-03-11 18:20:56 syllogismrxs>
///
/// @version 1.0
/// Created: 01 Feb 2013
///
/// ----------------------------------------------------------------------------
/// @section LICENSE
/// 
/// The MIT License (MIT)  
/// Copyright (c) 2012 Kevin DeMarco
///
/// Permission is hereby granted, free of charge, to any person obtaining a 
/// copy of this software and associated documentation files (the "Software"), 
/// to deal in the Software without restriction, including without limitation 
/// the rights to use, copy, modify, merge, publish, distribute, sublicense, 
/// and/or sell copies of the Software, and to permit persons to whom the 
/// Software is furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in 
/// all copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
/// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
/// DEALINGS IN THE SOFTWARE.
/// ----------------------------------------------------------------------------
/// @section DESCRIPTION
/// 
/// The MotionModels class ...
/// 
/// ----------------------------------------------------------------------------
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

#include <cv.h>

using namespace boost::numeric::odeint;

namespace syllo {

////////////
///     typedef std::vector< double > state_type;
///
///     /* The rhs of x' = f(x) defined as a class */
///     class harm_osc {
///
///	  double m_gam;
///
///     public:
///	  harm_osc( double gam ) : m_gam(gam) { }
///	  
///	  void operator() ( const state_type &x , state_type &dxdt , const double /* t */ )
///	       {
///		    dxdt[0] = x[1];
///		    dxdt[1] = -x[0] - m_gam*x[1];
///	       }
///     };
///
///////////
     #define SYLLO_PI 3.14159265359
     

     //typedef std::vector< double > state_type;
     typedef boost::array< double, 1 > state_1d_type;
     typedef boost::array< double , 3 > state_3d_type;
     
     
     typedef boost::numeric::odeint::runge_kutta4< state_3d_type > stepper_3d_type;

     //void write_cout( const double &x , const double t );
     void write_cout( const double &x , const double t );

     void write_lorenz( const state_3d_type &x , const double t );


     void lorenz( const state_3d_type &x , state_3d_type &dxdt , double t );
     void simple_rhs( const double x , double &dxdt , const double t );
     void cart_model(const state_3d_type &x , state_3d_type &dxdt , double t);

     void cart_trajectory(double t0, double dt, double tend,
			  std::vector<double> &xPos, 
			  std::vector<double> &yPos, 
			  std::vector<double> &heading,
			  std::vector<cv::Point2f> &cart_pos);

     void addMotionNoise(std::vector<cv::Point2f> &input,
			 std::vector<cv::Point2f> &output);

//class MotionModels {
//private:
//     int count_;
//     
//protected:
//public:
//     MotionModels();
//
//     int count();
//     void set_count(int count);
//};

}

#endif
