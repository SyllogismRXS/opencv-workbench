#include <iostream>
#include <math.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include "MotionModels.h"

using std::cout;
using std::endl;

using namespace boost::numeric::odeint;

namespace syllo {

     void write_cout( const double &x , const double t )
     {
	  cout << t << '\t' << x << endl;
     }

     void write_lorenz( const state_3d_type &x , const double t)
     {
	  cout << t << '\t' << x[0] << '\t' << x[1] << '\t' << x[2] << endl;
     }

     
     ///void test() 
     ///{
     ///	  state_type x;
     ///	  runge_kutta4< state_type > stepper;
     ///	  harm_osc osc(0.15);
     ///	  integrate_const( stepper ,  osc, x , 0.0 , 10.0 , 0.01 );
     ///	  const double dt = 0.01;
     ///	  for( double t = 0.0 ; t < 10.0 ; t += dt ) {
     ///	       stepper.do_step(  osc, x , t , dt );
     ///	  }
     ///}


     ///void lorenz_trajectory(std::vector<>)
     ///{     	  
     ///}

     
     //void integrate() 
     //{
     //	  integrate( syllo::lorenz , x , 0.0 , 25.0 , 0.1 , syllo::write_lorenz );
     //}
     
     void cart_model(const state_3d_type &x , state_3d_type &dxdt , double t)
     {
	  /// 0 : x-position
	  /// 1 : y-position
	  /// 2 : theta
	  
	  double u = 2;
	  double u_theta = SYLLO_PI/20;
	  //double u_theta = 0;
	  double L = 3;
	  dxdt[0] = u*cos(x[2]); 
	  dxdt[1] = u*sin(x[2]); 
	  dxdt[2] = u/L*tan(u_theta);
     }

     struct streaming_observer
     {
	  std::ostream &m_out;
	  streaming_observer( std::ostream &out ) : m_out( out ) {}

	  void operator()( const state_3d_type &x , double t ) const
	       {
		    m_out << "Step - t:" << t;
		    for( size_t i=0 ; i < x.size() ; ++i )
			 m_out << "\tx[" << i << "]:" << x[i];
		    m_out << "\n";
	       }
     };

     void cart_trajectory(double t0, double dt, double tend,
			  std::vector<double> &xPos, 
			  std::vector<double> &yPos, 
			  std::vector<double> &heading,
			  std::vector<cv::Point2f> &cart_pos_truth)
     {
	  //state_3d_type x = {0,0,SYLLO_PI/4};
	  state_3d_type x = {0,0,0};
	  //typedef controlled_runge_kutta< runge_kutta_cash_karp54< state_3d_type > > stepper_type;
	  //integrate_adaptive( stepper_type() , cart_model , x , t0 , tend , dt , streaming_observer( cout ) );
	  //integrate_const( runge_kutta4< state_3d_type >() , cart_model , x , t0 , tend , dt , streaming_observer( cout) );

	  //integrate_const( runge_kutta4< state_3d_type >() , lorenz , x , t0 , tend , dt , streaming_observer( cout) );

	  ///state_3d_type x = {0,0,0};
     	  runge_kutta4< state_3d_type > stepper;
	  /////integrate_const( stepper ,  cart_model, x , t0 , tend , dt );
	  ///
	  
	  //typedef controlled_runge_kutta< state_3d_type > controlled_stepper_type;
	  //controlled_stepper_type controlled_stepper;
          
	  for( double t = t0 ; t < tend ; t += dt ) {
	       xPos.push_back(x[0]);
	       yPos.push_back(x[1]);
	       heading.push_back(x[2]*180/SYLLO_PI);
	       cart_pos_truth.push_back(cv::Point2f(x[0], x[1]));
	       //cart_pos_truth.push_back(cv::Point2f(x[0], x[2]));
     	       //stepper.do_step(cart_model, x , t , dt );

	       //cout << "t:" << t << "\t" << x[0] << "\t" << x[1] << "\t" << x[2] << endl;

	       //stepper.do_step(lorenz, x , t , dt );
	       stepper.do_step(cart_model, x , t , dt );
	       //controlled_stepper.do_step(cart_model, x , t , dt );
	  }
     }

     void lorenz( const state_3d_type &x , state_3d_type &dxdt , double t )
     {
	  const double sigma = 10.0;
	  const double R = 28.0;
	  const double b = 8.0 / 3.0;

	  dxdt[0] = sigma * ( x[1] - x[0] );
	  dxdt[1] = R * x[0] - x[1] - x[0] * x[2];
	  dxdt[2] = -b * x[2] + x[0] * x[1];
     }

     void simple_rhs( const double x , double &dxdt , const double t )
     {
	  dxdt = 3.0/(2.0*t*t) + x/(2.0*t);
     }

     void addMotionNoise(std::vector<cv::Point2f> &input,
			 std::vector<cv::Point2f> &output)
     {
	  boost::mt19937 rng;
	  boost::normal_distribution<> nd(0.0, 1.0);
	  boost::variate_generator<boost::mt19937&, 
				   boost::normal_distribution<> > var_nor(rng, nd);

	  std::vector<cv::Point2f>::iterator it;
	  for (it = input.begin() ; it != input.end() ; it++) {
	            //double value = var_nor() + (*it);
	       cv::Point2f newPoint;
	       newPoint.x = it->x + var_nor();
	       newPoint.y = it->y + var_nor();
	       output.push_back(newPoint);
	  }
     }

}
