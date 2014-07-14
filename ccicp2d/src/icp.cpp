/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

Authors: Andreas Geiger

libicp is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libicp is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libicp; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

#include "ccicp2d/icp.h"
#include <ros/ros.h>

using namespace std;

Icp::Icp (double *M_GA, double *M_NGA ,const int32_t M_GA_num,const int32_t M_NGA_num,const int32_t dim) :
  dim(dim), sub_step(10), max_iter(20), min_delta(1e-6) {
  
  // check for correct dimensionality
  if (dim!=2 && dim!=3) {
    ROS_ERROR_STREAM("LIBICP works only for data of dimensionality 2 or 3" );
    M_tree_GA = 0;
    M_tree_NGA = 0;
    return;
  }
  
  // check for minimum number of points
  if ((M_GA_num + M_NGA_num)<5) {
    ROS_ERROR_STREAM("LIBICP works only with at least 5 model points");
    M_tree_GA = 0;
    M_tree_NGA = 0;
    return;
  }
  
  //copy the number of points to the class
  
   M_GA_SIZE =M_GA_num;
   M_NGA_SIZE =M_NGA_num;

  // copy model points to M_data
  M_data_GA.resize(boost::extents[M_GA_num][dim]);
  for (int32_t m=0; m<M_GA_num; m++)
    for (int32_t n=0; n<dim; n++)
      M_data_GA[m][n] = (float)M_GA[m*dim+n];

 // copy model points to M_data
  M_data_NGA.resize(boost::extents[M_NGA_num][dim]);
  for (int32_t m=0; m<M_NGA_num; m++)
    for (int32_t n=0; n<dim; n++)
      M_data_NGA[m][n] = (float)M_NGA[m*dim+n];

  // build a kd tree from the model point cloud
  
  //if(M_GA_num>0)
	M_tree_GA = new kdtree::KDTree(M_data_GA);
	
 // build a kd tree from the model point cloud
  //if(M_NGA_num>0)
	M_tree_NGA = new kdtree::KDTree(M_data_NGA);
}

Icp::~Icp () {
  if (M_tree_GA)
    delete M_tree_GA;

if (M_tree_NGA)
    delete M_tree_NGA;
}

void Icp::fit (double *T_GA,double *T_NGA,const int32_t T_GA_num,const int32_t T_NGA_num,Matrix &R,Matrix &t,double indist, double h_dist) {
  
  // make sure we have a model tree
  /*if (!M_tree_GA || !M_tree_GA) {
    ROS_ERROR_STREAM( "ERROR: Model not initialized");
    return;
  }*/
  
  // check for minimum number of points
  if (T_NGA_num<5) {
    ROS_WARN_STREAM("Template has " << T_NGA_num <<" NGA points" );
    //return;
  }

   // check for minimum number of points
  if (T_GA_num<5) {
    ROS_WARN_STREAM("Warning: Template has "<< T_GA_num << " GA points");
    //return;
  }
  
  if (T_GA_num + T_NGA_num < 5) {
    ROS_ERROR_STREAM("ERROR: Total template has " << T_GA_num + T_NGA_num <<" points");
    return;
  }



  //vector<int32_t> active_GA;
  //active_GA.clear();

  //vector<int32_t> active_NGA;
  //active_NGA.clear();
 
  fitIterate(T_GA,T_NGA,T_GA_num,T_NGA_num,R,t,indist);
}

void Icp::fitIterate(double *T_GA,double *T_NGA,const int32_t T_GA_num,const int32_t T_NGA_num,Matrix &R,Matrix &t,double inDist) {
     
  // iterate until convergence
  for (int32_t iter=0; iter<max_iter; iter++)
    if (fitStep(T_GA,T_NGA,T_GA_num,T_NGA_num,R,t,inDist)<min_delta)
      break;
}


