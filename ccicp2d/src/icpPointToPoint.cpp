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

#include "ccicp2d/icpPointToPoint.h"
#include <ros/ros.h>

struct customPt
{
double x;
double y;
};

using namespace std;

// Also see (3d part): "Least-Squares Fitting of Two 3-D Point Sets" (Arun, Huang and Blostein)
double IcpPointToPoint::fitStep (double *T_GA,double *T_NGA,const int32_t T_GA_num,const int32_t T_NGA_num,Matrix &R,Matrix &t, double inDist) {
  // cout<<"In Fit Step"<<endl;

  // kd tree query + result
  std::vector<float>	query(dim);
  kdtree::KDTreeResultVector result;

 int totalSize = T_GA_num +  T_NGA_num;
 int inIDX=0;
 


 // init matrix for point correspondences
  Matrix p_m(totalSize,dim); // model
  Matrix p_t(totalSize,dim); // template
      
  // init mean
  Matrix mu_m(1,dim);
  Matrix mu_t(1,dim);
  
    // extract matrix and translation vector
    double r00 = R.val[0][0]; double r01 = R.val[0][1];
    double r10 = R.val[1][0]; double r11 = R.val[1][1];
    double t0  = t.val[0][0]; double t1  = t.val[1][0];

	
	if(M_GA_SIZE>3)
	{
		// establish correspondences for GA
		for (int32_t i=0; i<T_GA_num; i++) 
		{

		  // get index of active point
		  int32_t idx = i;

		  // transform point according to R|t
		  query[0] = r00*T_GA[idx*2+0] + r01*T_GA[idx*2+1] + t0;
		  query[1] = r10*T_GA[idx*2+0] + r11*T_GA[idx*2+1] + t1;

		  // search nearest neighbor
		  M_tree_GA->n_nearest(query,1,result);

		 //check for outlier (add height thing here too)
		 if (result[0].dis<inDist)
		 {
			  // set model point
			  p_m.val[inIDX][0] = M_tree_GA->the_data[result[0].idx][0]; mu_m.val[0][0] += p_m.val[inIDX][0];
			  p_m.val[inIDX][1] = M_tree_GA->the_data[result[0].idx][1]; mu_m.val[0][1] += p_m.val[inIDX][1];

			  // set template point
			  p_t.val[inIDX][0] = query[0]; mu_t.val[0][0] += p_t.val[inIDX][0];
			  p_t.val[inIDX][1] = query[1]; mu_t.val[0][1] += p_t.val[inIDX][1];
			  inIDX++;

		  }

		 
		}
	}
	
	if(M_NGA_SIZE>3)
	{
   
	   // establish correspondences for NGA
		for (int32_t i=0; i<T_NGA_num; i++) 
		{

		  // get index of active point
		  int32_t idx = i;

		  // transform point according to R|t
		  query[0] = r00*T_NGA[idx*2+0] + r01*T_NGA[idx*2+1] + t0;
		  query[1] = r10*T_NGA[idx*2+0] + r11*T_NGA[idx*2+1] + t1;

		  // search nearest neighbor
		  M_tree_NGA->n_nearest(query,1,result);

		 //check for outlier (add height thing here too)
		 if (result[0].dis<inDist)
		 {
			  // set model point
			  p_m.val[inIDX][0] = M_tree_NGA->the_data[result[0].idx][0]; mu_m.val[0][0] += p_m.val[inIDX][0];
			  p_m.val[inIDX][1] = M_tree_NGA->the_data[result[0].idx][1]; mu_m.val[0][1] += p_m.val[inIDX][1];

			  // set template point
			  p_t.val[inIDX][0] = query[0]; mu_t.val[0][0] += p_t.val[inIDX][0];
			  p_t.val[inIDX][1] = query[1]; mu_t.val[0][1] += p_t.val[inIDX][1];
			  inIDX++;

		  }
		  
		}
     
    }
	
  if(inIDX == 0) {
    ROS_WARN_STREAM("NO CORRESPONDENCES!!");
    return -1;
  }

 //resize the matrices
 
 p_m.m = inIDX;
 p_t.m = inIDX;
 // cout<<"resize mat"<<endl;

//save the number of correspondences

numCorr = inIDX;	

//set the point correspondences so we can calculate the edge weights

cModelPoints = p_m;
cScenePoints = p_t;
   
  // subtract mean
  mu_m = mu_m/(double)inIDX;
  mu_t = mu_t/(double)inIDX;



  Matrix q_m = p_m - Matrix::ones(inIDX,1)*mu_m;
  Matrix q_t = p_t - Matrix::ones(inIDX,1)*mu_t;


  // compute relative rotation matrix R and translation vector t
  Matrix H = ~q_t*q_m;
  Matrix U,W,V;
  H.svd(U,W,V);
  Matrix R_ = V*~U;
  Matrix t_ = ~mu_m - R_*~mu_t;
  
  // compose: R|t = R_|t_ * R|t
  R = R_*R;
  t = R_*t+t_;

  // return max delta in parameters
  if (dim==2) return max((R_-Matrix::eye(2)).l2norm(),t_.l2norm());
  else        return max((R_-Matrix::eye(3)).l2norm(),t_.l2norm());
}

/*std::vector<int32_t> IcpPointToPoint::getInliers (double *T,const int32_t T_num,const Matrix &R,const Matrix &t,const double indist){

  // init inlier vector + query point + query result
  vector<int32_t>            inliers;
  std::vector<float>         query(dim);
  kdtree::KDTreeResultVector neighbor;
  
  // dimensionality 2
  if (dim==2) {
  
    // extract matrix and translation vector
    double r00 = R.val[0][0]; double r01 = R.val[0][1];
    double r10 = R.val[1][0]; double r11 = R.val[1][1];
    double t0  = t.val[0][0]; double t1  = t.val[1][0];

    // check for all points if they are inliers
    for (int32_t i=0; i<T_num; i++) {

      // transform point according to R|t
      query[0] = r00*T[i*2+0] + r01*T[i*2+1] + t0;
      query[1] = r10*T[i*2+0] + r11*T[i*2+1] + t1;

      // search nearest neighbor
      M_tree->n_nearest(query,1,neighbor);

      // check if it is an inlier
      if (neighbor[0].dis<indist)
        inliers.push_back(i);
    }
    
  // dimensionality 3
  } else {
    
    // extract matrix and translation vector
    double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
    double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
    double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
    double t0  = t.val[0][0]; double t1  = t.val[1][0]; double t2  = t.val[2][0];

    // check for all points if they are inliers
    for (int32_t i=0; i<T_num; i++) {

      // transform point according to R|t
      query[0] = r00*T[i*3+0] + r01*T[i*3+1] + r02*T[i*3+2] + t0;
      query[1] = r10*T[i*3+0] + r11*T[i*3+1] + r12*T[i*3+2] + t1;
      query[2] = r20*T[i*3+0] + r21*T[i*3+1] + r22*T[i*3+2] + t2;

      // search nearest neighbor
      M_tree->n_nearest(query,1,neighbor);

      // check if it is an inlier
      if (neighbor[0].dis<indist)
        inliers.push_back(i);
    }
  }
  
  // return vector with inliers
  return inliers;
}*/
void IcpPointToPoint::getEdgeWeight(double* eW)
{

	int numCorr = cModelPoints.m; //number of correspondences
	
	double x=0; double y=0; double dx=0; double dy=0;double sx=0; double sy=0;double xpy=0;double xy=0;
	double ax,ay,bx,by;
	double tx=0; double ty=0;

	Matrix C(3,3);
	Matrix MM(3,3);
	Matrix MMi(3,3);
	Matrix MZ(3,1);
	MM.zero();
	MZ.zero();
	MMi.zero();

	double ss=0;
	
	for(int i=0; i<numCorr; i++)
	{
		ax = cModelPoints.val[i][0];
		ay = cModelPoints.val[i][1];
		bx = cScenePoints.val[i][0];
		by = cScenePoints.val[i][1];

		x = (ax + bx)/2.0;
      		y = (ay + by)/2.0;
      		dx = ax - bx;
		dy = ax - bx;

		 // Sum up all necessary values to construct MM
      		sx += x;
      		sy += y;
       		xpy += x*x + y*y;
     		xy += x*y;

		 // Sum up each part of MZ
	      MZ.val[0][0] += dx;
	      MZ.val[1][0] += dy;
	      MZ.val[2][0] += -y * dx + x * dy;
	
	}
		// Now construct the symmetrical matrix MM
		MM.val[0][0] = numCorr;
		MM.val[1][1] = numCorr;
		MM.val[2][2] = xpy;
		MM.val[0][2] = -sy; MM.val[2][0] = -sy;
		MM.val[1][2] = sx; MM.val[2][1] = sx;

		MMi = MM;
		MMi.inv();
		Matrix D = MMi*MZ;

	//iterate through again to get ss

	for(int i=0; i<numCorr; i++)
	{

		ax = cModelPoints.val[i][0];
		ay = cModelPoints.val[i][1];
		bx = cScenePoints.val[i][0];
		by = cScenePoints.val[i][1];

		x = (ax + bx)/2.0;
      		y = (ay + by)/2.0;

		
		tx = (ax - bx - D.val[0][0] +y*D.val[2][0]);
		ty = (ay - by - D.val[1][0] -x*D.val[2][0]);

		ss += tx*tx+ty*ty;

	}
		ss = ss/(2*numCorr - 3);
		double sconst= (1.0/ss);
		C = MM*sconst;

		//push out C using the edge weights
		eW[0] = C.val[0][0];eW[1] = C.val[0][1];eW[2] = C.val[0][2];
		eW[3] = C.val[1][0];eW[4] = C.val[1][1];eW[5] = C.val[1][2];
		eW[6] = C.val[2][0];eW[7] = C.val[2][1];eW[8] = C.val[2][2];

}
int IcpPointToPoint::getNumberCorrespondences(void)
{
	return numCorr;
}
