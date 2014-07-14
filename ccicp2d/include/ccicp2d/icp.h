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

#ifndef ICP_H
#define ICP_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <vector>

#include "ccicp2d/matrix.h"
#include "ccicp2d/kdtree.h"

class Icp {

public:

  // constructor
  // input: M_GA ....... pointer to first model point (ground adjacent)
  //	    M_NGA ....... pointer to first model point (non ground adjacent)
  //        M_num ... number of model points
  //        dim   ... dimensionality of model points (2 or 3)
  Icp (double *M_GA, double *M_NGA ,const int32_t M_GA_num,const int32_t M_NGA_num,const int32_t dim);
  
  // deconstructor
  virtual ~Icp ();
  
  // set subsampling step size of coarse matching (1. stage)
  void setSubsamplingStep (int32_t val) { sub_step  = val; }
  
  // set maximum number of iterations (1. stopping criterion)
  void setMaxIterations   (int32_t val) { max_iter  = val; }
  
  // set minimum delta of rot/trans parameters (2. stopping criterion)
  void setMinDeltaParam   (double  val) { min_delta = val; }
  
  // fit template to model yielding R,t (M = R*T + t)
  // input:  T ....... pointer to first template point
  //         T_num ... number of template points
  //         R ....... initial rotation matrix
  //         t ....... initial translation vector
  //         indist .. inlier distance (if <=0: use all points)
  // output: R ....... final rotation matrix
  //         t ....... final translation vector
  //	     h_thresh	height threshold
  void fit(double *T_GA,double *T_NGA,const int32_t T_GA_num,const int32_t T_NGA_num,Matrix &R,Matrix &t,double indist, double h_dist);

  void getEdgeWeight(double* eW);
  int getNumberCorrespondences(void);

  //for getting out the covariance (information) of a match

  
private:
  
  // iterative fitting
  void fitIterate(double *T_GA,double *T_NGA,const int32_t T_GA_num,const int32_t T_NGA_num,Matrix &R,Matrix &t,double indist);
  
  // inherited classes need to overwrite these functions
  virtual double               fitStep(double *T_GA,double *T_NGA,const int32_t T_GA_num,const int32_t T_NGA_num,Matrix &R,Matrix &t, double inDist) = 0;
 // virtual std::vector<int32_t> getInliers(double *T_GA,double *T_NGA,const int32_t T_GA_num,const int32_t T_NGA_num,const Matrix &R,const Matrix &t,const double indist) = 0;
  
protected:
  
  // kd tree of model points
  kdtree::KDTree*     M_tree_GA;
  kdtree::KDTreeArray M_data_GA;
  kdtree::KDTree*     M_tree_NGA;
  kdtree::KDTreeArray M_data_NGA;
  
  int M_GA_SIZE;
  int M_NGA_SIZE;
  
  int32_t dim;       // dimensionality of model + template data (2 or 3)
  int32_t sub_step;  // subsampling step size
  int32_t max_iter;  // max number of iterations
  double  min_delta; // min parameter delta
  Matrix cModelPoints;
  Matrix cScenePoints;
  int numCorr;

 
};

#endif // ICP_H
