#include <ros/ros.h>
#include <iostream>
#include "mls/mls.h"
#include <Eigen/Eigenvalues>
#include <cassert>
#include <algorithm>
#include <tf/tf.h>

#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <eigen_conversions/eigen_msg.h>

#include <visualization_msgs/MarkerArray.h>

using namespace std;

void MLS::clearMap(){
    for(int i = 0; i < size_x; i++) {
        for(int j =0; j < size_y; j++) {
            Cell* cell = grid(i,j);
            cell->cloud.clear();
            cell->clusters.clear();
            cell->drivable = -1;
            cell->updated = false;
            drivabilityGrid->data[i+size_x*j] = -1; 
        }
    }

    global_cloud->clear();
}


void MLS::addToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, geometry_msgs::PoseStamped pose)
{
    setPose(pose);

    if(rolling) {
        //transform cloud to account of discritization error
        //and orientation to global frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Affine3d trans;
        geometry_msgs::PoseStamped transPose = pose;
        transPose.pose.position.x = curPose.pose.position.x - pose.pose.position.x; 
        transPose.pose.position.y = curPose.pose.position.y - pose.pose.position.y; 
        tf::poseMsgToEigen(transPose.pose, trans);
        pcl::transformPointCloud(*input_cloud,*trans_cloud, trans); //transform to global orientation
        //addToMap(trans_cloud);
        addToOccupancy(trans_cloud);
    } else {
        addToMap(input_cloud);
    }
}

//comparison function for sort
bool cluster_comp(Cluster i,Cluster j) { return (i.mean[2]<j.mean[2]); }


void MLS::addToOccupancy(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {

    //segment cloud
    pcl::PointCloud<PointXYZGD>::Ptr drv_cloud (new pcl::PointCloud<PointXYZGD>);
    pcl::PointCloud<PointXYZGD>::Ptr ground_cloud (new pcl::PointCloud<PointXYZGD>);
    pcl::PointCloud<PointXYZGD>::Ptr obs_cloud (new pcl::PointCloud<PointXYZGD>);

    gSeg.setupGroundSegmentation(input_cloud,ground_cloud,obs_cloud,drv_cloud);
	gSeg.segmentGround();

    //run through non-drivable
    int offset_x = size_x/2;
    int offset_y = size_y/2;
    Cell* cell; 
    for(int i = 0; i < drv_cloud->size(); i++) {
    
        //find grid index
        PointXYZGD& pt = drv_cloud->points[i];
        int x = (int)(pt.x/resolution + offset_x);
        int y = (int)(pt.y/resolution + offset_y);

        double rng;
        if(rolling) {
            rng = sqrt(pt.x*pt.x + pt.y*pt.y);
        } else {
            double rx = curPose.pose.position.x - pt.x;
            double ry = curPose.pose.position.y - pt.y;
            rng = sqrt(rx*rx + ry*ry);
        }

        //check range
        if(x < 0 || y < 0 || x >= size_x || y >= size_x || rng > max_range) continue;


        cell = grid(x,y);
        //add to grid Cell
        if(cell->clusters.empty()) {
            cell->clusters.push_back(Cluster());
        }

        cell->clusters[0].num_pts += occupancy_increment;

        if(cell->clusters[0].num_pts > min_cluster_points) {
            //cell is occupied
            cell->drivable = 0;
            drivabilityGrid->data[x+size_x*y] = 100;
        }
    }

    
    //run through ground
    for(int i = 0; i < ground_cloud->size(); i++) {
    
        //find grid index
        PointXYZGD& pt = ground_cloud->points[i];
        int x = (int)(pt.x/resolution + offset_x);
        int y = (int)(pt.y/resolution + offset_y);

        double rng;
        if(rolling) {
            rng = sqrt(pt.x*pt.x + pt.y*pt.y);
        } else {
            double rx = curPose.pose.position.x - pt.x;
            double ry = curPose.pose.position.y - pt.y;
            rng = sqrt(rx*rx + ry*ry);
        }

        //check range
        if(x < 0 || y < 0 || x >= size_x || y >= size_x || rng > max_range) continue;


        cell = grid(x,y);
        //add to grid Cell
        if(cell->clusters.empty()) {
            cell->clusters.push_back(Cluster());
        }
        cell->clusters[0].num_pts -= occupancy_decrement;

        if(cell->clusters[0].num_pts < min_cluster_points) {
            //cell is occupied
            cell->drivable = 1;
            drivabilityGrid->data[x+size_x*y] = 0;
        }
    }

    if(!disable_pointcloud) {
        //we should probably filter this
        pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*drv_cloud, *output);
       *global_cloud += *output; //this could get very large!! make usre to use filterCloud method
    }
}

void MLS::updateCell(int x, int y) {

    Cell* cell = grid(x,y);
    cell->updated = false;

    //recalculate mean/covar for drivability
    for(int j = 0; j < cell->cloud.size(); j++) {
        const pcl::PointXYZ &pt = cell->cloud[j];

        //Find corresponding cluster
        int cluster_idx = -1;
        double uninit_dist = 100000;
        int uninit_idx = -1;
        for(int c = 0; c < cell->clusters.size(); c++) {
            double cur_dist = abs(cell->clusters[c].mean[2] - pt.z);

            if(cell->clusters[c].num_pts < min_cluster_points) {
                if(cur_dist < uninit_dist) {
                    uninit_dist = cur_dist;
                    uninit_idx = c;
                }
            } else {
                //find if it falls in a cluster
                if(cur_dist < (sqrt(cell->clusters[c].cov(2,2))*cluster_sigma_factor+cluster_dist_threshold)) {
                    cluster_idx = c;
                    break;
                }
            }
        }
        //check for threshold on new cluster
        if(cell->clusters.empty() || cluster_idx == -1) {
            //check for uninitialized clusters
            if(uninit_idx == -1 || uninit_dist > robot_height) {
                //add new cluster
                if(cell->clusters.size() < max_clusters) {
                    cell->clusters.push_back(Cluster()); 
                    cluster_idx = cell->clusters.size()-1;
                } else {
                    //to many clusters
                    //FIXME how to de deal with this?
                    continue;
                }
            } else {
                cluster_idx = uninit_idx;
            }
        } 

        Cluster* cluster = &cell->clusters[cluster_idx]; 

        //cap the max number of points -> allows us to remove obstacles more easily
        if(cluster->num_pts == max_cluster_points) {
            //remove points from other clusters
            for(int k = 1; k < cell->clusters.size(); k++) {
                if(cluster_idx != k) { //is not current cluster
                    cell->clusters[k].num_pts--;
                    if(cell->clusters[k].num_pts <= 0) {
                        cell->clusters.erase(cell->clusters.begin()+k); //remove cluster
                    }
                }
            }
            
        } else {
            cluster->num_pts++;
        }

        cluster->mean[0] = (((cluster->num_pts) - 1)/(cluster->num_pts))*(cluster->mean[0]) 
                            + 1/(cluster->num_pts)*pt.x; //FIXME x/y mean be broke for moving cells
        cluster->mean[1] = (((cluster->num_pts) - 1)/(cluster->num_pts))*(cluster->mean[1])
                            + 1/(cluster->num_pts)*pt.y;
        cluster->mean[2] = (((cluster->num_pts) - 1)/(cluster->num_pts))*(cluster->mean[2]) 
                            + 1/(cluster->num_pts)*pt.z;
          
        // cov(0,0) += pt.x*pt.x;
        //   
        // cov(1,0) += pt.y*pt.x;
        // cov(1,1) += pt.y*pt.y;
        //   
        // cov(2,0) += pt.z*pt.x;
        // cov(2,1) += pt.z*pt.y;*/
        if(cluster->num_pts > 1) {
            cluster->cov(2,2) = ((cluster->num_pts - 1)/(cluster->num_pts))*(cluster->cov(2,2)) 
                            + 1.0/(cluster->num_pts - 1)*(pt.z - cluster->mean[2])*(pt.z - cluster->mean[2]);    
            cluster->cov(2,2) = max(cluster->cov(2,2), 0.001); //limit the thinness of the ground 3cm stdev FIXME magic number
        } else { //new cluster was added
            std::sort(cell->clusters.begin(),cell->clusters.end(), cluster_comp);
        }
    }

    int ground_idx = -1;
    for(int c = 0; c < cell->clusters.size(); c++) {
        if(cell->clusters[c].num_pts > min_cluster_points) {
            ground_idx = c;
            break;
        }
    }
    if(ground_idx == -1) {
        //no cluster with minimum points
        return;
    }

    cell->cloud.clear();
      

    //OLD CODE
    //mirror covariance
    // for (int k = 0; k < 3; k++) {
    //     for (int l = 0; l <= k; l++) 
    //     {
    //         cov(l,k) = cov(k,l);
    //     }
    // }

    // Compute the SVD (covariance matrix is symmetric so U = V')
    //Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU);
    //grid(x,y)->U = svd.matrixU ();
    //grid(x,y)->e = svd.singularValues();

    //check eigen values are not singular
    //if(grid(x,y)->e[1] > SINGULAR_THRESHOLD) {
    //    //check for "upward" normal
    //    double normal_angle = acos(abs(grid(x,y)->U(2,2)));s
    //  //  if(normal_angle > normal_threshold) {
    //                     grid(x,y)->drivable = 0;
    //        drivabilityGrid->data[x+size_x*y] = 100;
    //        return;
    //    }
    // } 


     //check intracell cluster distance
     if(ground_idx+1 < cell->clusters.size() && cell->clusters[ground_idx+1].num_pts > min_cluster_points) {
          double clearance = cell->clusters[ground_idx+1].mean[2] - sqrt(cell->clusters[ground_idx+1].cov(2,2))*2 
                                - cell->clusters[ground_idx].mean[2]; //2 sigma clearance
          if(clearance < cluster_combine_dist) {
              
              //combine clusters
              double ratio0 = cell->clusters[ground_idx].num_pts/(cell->clusters[ground_idx].num_pts + cell->clusters[ground_idx+1].num_pts);
              double ratio1 = cell->clusters[ground_idx+1].num_pts/(cell->clusters[ground_idx].num_pts + cell->clusters[ground_idx+1].num_pts);
              cell->clusters[ground_idx].mean[0] = ratio0*(cell->clusters[ground_idx].mean[0]) 
                                            + ratio1*(cell->clusters[ground_idx+1].mean[0]);
              cell->clusters[ground_idx].mean[1] = ratio0*(cell->clusters[ground_idx].mean[1]) 
                                            + ratio1*(cell->clusters[ground_idx+1].mean[1]);
              cell->clusters[ground_idx].mean[2] = ratio0*(cell->clusters[ground_idx].mean[2]) 
                                            + ratio1*(cell->clusters[ground_idx+1].mean[2]);
              cell->clusters[ground_idx].cov(2,2) = (ratio0*cell->clusters[ground_idx].cov(2,2)
                                            + ratio1*cell->clusters[ground_idx+1].cov(2,2)); //FIXME??
              cell->clusters.erase(cell->clusters.begin()+ground_idx+1);
          } else if(clearance < drive_dist_threshold) {
            cell->drivable = 0;
            drivabilityGrid->data[x+size_x*y] = 100;
            return;
          }
     }


    double max_diff = 0;
    //check for intercell height diff to neighbour cells
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            if( (i==0 && j==0) || i+x<0 || i+x>=size_x || j+y<0 || j+y>=size_y) continue;
            if(grid(x+i,y+j)->updated == true) {
               updateCell(x+i, y+j); //OH GOD ITS RECURSIVE!!!
            }
            if( !grid(x+i,y+j)->clusters.empty() && grid(x+i,y+j)->clusters[0].num_pts > min_cluster_points) {
                double ndiff = cell->clusters[0].mean[2] - grid(x+i,y+j)->clusters[0].mean[2];
                
                if( ndiff > height_threshold) {
                    cell->drivable = 0;
                    drivabilityGrid->data[x+size_x*y] = 100;
                    return;
                }
                if(ndiff > max_diff) {
                    max_diff = ndiff;
                }
            }
        }
    }


    //check covariance
     if(abs(cell->clusters[ground_idx].cov(2,2)) > normal_threshold) { 
            cell->drivable = 0;
            drivabilityGrid->data[x+size_x*y] = 100;
            return;
     }


   //drivability test passed
   cell->drivable = 1;
   drivabilityGrid->data[x+size_x*y] = 0;
}


void MLS::addToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
    //cout << " Adding a cloud " << endl;
    // NOTE: setPose(...) must be called before this frunction!
    //For rolling map we assume cloud has global orientation and is centered at (0,0)
    //For global map we assume that the cloud has been translated into the global frame

    int offset_x = size_x/2;
    int offset_y = size_y/2;

    for(int i = 0; i < input_cloud->size(); i++) {
    
        //find grid index
        pcl::PointXYZ pt = input_cloud->points[i];
        int x = (int)(pt.x/resolution + offset_x);
        int y = (int)(pt.y/resolution + offset_y);

        double rng;
        if(rolling) {
            rng = sqrt(pt.x*pt.x + pt.y*pt.y);
        } else {
            double rx = curPose.pose.position.x - pt.x;
            double ry = curPose.pose.position.y - pt.y;
            rng = sqrt(rx*rx + ry*ry);
        }

        //check range
        if(x < 0 || y < 0 || x >= size_x || y >= size_x || rng > max_range) continue;

        //add to grid Cell
        grid(x,y)->cloud.push_back(pt);

        //flag update
        grid(x,y)->updated = true;
    }

    int curX, curY;
    if(rolling) {
        //rolling is always centered
        curX = offset_x;
        curY = offset_y;
    } else {
        curX = (int)(curPose.pose.position.x/resolution + offset_x);
        curY = (int)(curPose.pose.position.y/resolution + offset_y);
    }
    for(int i = -update_dist; i < update_dist; i++) {
        for(int j =-update_dist; j < update_dist; j++) {
            int x = i + curX;
            int y = j + curY;
            if(x < 0 || y < 0 || x >= size_x || y >= size_y) continue;  
            if(grid(x,y)->updated == true) {
                updateCell(x,y);
            }
        }
    }

    if(!disable_pointcloud) {
        //we should probably filter this
       *global_cloud += *input_cloud; //this could get very large!! make usre to use filterCloud method
    }
    
}

void MLS::setPose(geometry_msgs::PoseStamped pose)
{

    if(!rolling) {
        //don't need to move map just set curPose and return;
        curPose = pose;
        return;
    }

    //Shift Map
    
    double xdiff = pose.pose.position.x-curPose.pose.position.x;
    double ydiff = pose.pose.position.y-curPose.pose.position.y;
    
    //offset map so new pose is centered
    int dx = round(xdiff/resolution);
    int dy = round(ydiff/resolution);

    if(dx != 0 || dy != 0) {
        grid.shiftOrigin(dx,dy);

        curPose.pose.position.x += dx*resolution;
        curPose.pose.position.y += dy*resolution;

        if(!disable_pointcloud) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>); 

            *temp = *global_cloud;

            //transform and crop global_cloud
            Eigen::Matrix4f diff = Eigen::Matrix4f::Identity();
            diff(0,3) = -dx*resolution;
            diff(1,3) = -dy*resolution;
            pcl::transformPointCloud(*temp,*global_cloud,diff); //data is in global_cloud

            pcl::PassThrough<pcl::PointXYZ> pass;
            double min_size = min(size_x, size_y);
            double crop_dist = min_size*resolution/2;
            pass.setFilterLimits (-crop_dist,crop_dist);
            pass.setFilterFieldName ("x");
            pass.setInputCloud (global_cloud);
            pass.filter (*temp); //data is in temp

            pass.setFilterFieldName ("y");
            pass.setInputCloud (temp);
            pass.filter (*global_cloud); //data is in global_cloud
        }


        //Copy new map into Drivability Grid FIXME[5] could be more efficent??
        for (int i = 0; i < size_x; i++) {
            for (int j = 0; j < size_y; j++) {
                //clear shifted cells
                if(i < -dx || i >= size_x-dx || j < -dy || j >= size_y-dy) {
                    //clear cell
                    Cell* cell = grid(i,j);
                    cell->cloud.clear();
                    cell->clusters.clear();
                    cell->drivable = -1;
                    cell->updated = false;
                }
                //copy new shifted map
                if(grid(i,j)->drivable < 0) {
                    drivabilityGrid->data[i+size_x*j] =  -1; //unknown
                } else {
                    drivabilityGrid->data[i+size_x*j] =  (grid(i,j)->drivable == 0)?100:0;
                }
            }
        }
    }

}

void MLS::offsetMap(const geometry_msgs::PoseStamped& pose)
{
    for (int i = 0; i < size_x; i++) {
        for (int j = 0; j < size_y; j++) {
            Cell* cell = grid(i,j);
            if( !cell->clusters.empty() ) {
                for(int c = 0; c < cell->clusters.size(); c++) {
                    cell->clusters[c].mean[2] += pose.pose.position.z;
                }
            }
        }
    }

    if(!disable_pointcloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>); 
        *temp = *global_cloud;

        //transform and crop global_cloud
        Eigen::Matrix4f diff = Eigen::Matrix4f::Identity();
        diff(2,3) = pose.pose.position.z; 
        pcl::transformPointCloud(*temp,*global_cloud,diff); //data is in global_cloud
    }


}


void MLS::filterPointCloud(double xy, double z)
{
    //voxel filter map
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(global_cloud);
    sor.setLeafSize (xy, xy, z); 
    sor.filter (*temp); //data is in temp*/

    *global_cloud = *temp;
}

void  MLS::getSegmentedClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle,pcl::PointCloud<pcl::PointXYZ>::Ptr& ground) 
{
    obstacle_cloud->clear();
    ground_cloud->clear();

    int curX = (int)(curPose.pose.position.x/resolution + size_x/2);
    int curY = (int)(curPose.pose.position.y/resolution + size_y/2);
    for(int i = -update_dist + curX; i < update_dist+curX; i++) {
        for(int j =-update_dist+ curY; j < update_dist + curY; j++) {
            if(i < 0 || i >= size_x || j >= size_y || j < 0) continue;

            Cell* cell = grid(i,j); 
            //check for obstacle points
            if(cell->clusters.empty() ) continue; 

            //add means
            for(int c = 0; c < cell->clusters.size(); c++) {
                if(cell->clusters[c].num_pts >= min_cluster_points) {
                    pcl::PointXYZ pt;
                    pt.x = cell->clusters[c].mean[0];
                    pt.y = cell->clusters[c].mean[1];
                    pt.z = cell->clusters[c].mean[2];
                    if(cell->drivable == 0 || c > 0) {
                        obstacle_cloud->push_back(pt);
                    } else {
                        ground_cloud->push_back(pt);
                    }
                }
            }
            
        }
    }

    obstacle = obstacle_cloud;
    ground = ground_cloud;
}


void MLS::visualize(ros::Publisher pub, std::string frame) 
{
    static int prev_size = 0;
    
    visualization_msgs::MarkerArray marker_array;
	visualization_msgs::Marker marker;

	marker.header.frame_id = frame;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.ns = "mls";

	marker.action = visualization_msgs::Marker::DELETE;
	marker.lifetime = ros::Duration(0.01);

    //remove old clusters
    for(int x = 0; x < prev_size; x++) {
        marker_array.markers.push_back(marker);
    }

	marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0);

    int idx_count = 0;

    int curX, curY;
    if(rolling) {
        curX = size_x/2;
        curY = size_y/2;
    } else {
        curX = (int)(curPose.pose.position.x/resolution + size_x/2);
        curY = (int)(curPose.pose.position.y/resolution + size_y/2);
    }
    for(int i = -update_dist + curX; i < update_dist+curX; i++) {
        for(int j =-update_dist+ curY; j < update_dist + curY; j++) {
            if(i < 0 || i >= size_x || j >= size_y || j < 0) continue;

            Cell* cell = grid(i,j);

            int ground_idx = -1;
            for(int c = 0; c < cell->clusters.size(); c++) {
                if(cell->clusters[c].num_pts > min_cluster_points) {
                    ground_idx = c;
                    break;
                }
            }


            for(int c = 0; c < cell->clusters.size(); c++) {


                marker.id = idx_count;
                idx_count++;
                
                if(rolling) {
                    marker.pose.position.x = (i-curX)*resolution;//
                    marker.pose.position.y = (j-curY)*resolution;
                } else {
                    marker.pose.position.x = cell->clusters[c].mean[0];
                    marker.pose.position.y = cell->clusters[c].mean[1];
                }
                marker.pose.position.z = cell->clusters[c].mean[2];
                
                //FIXME calc actual cov
               // Eigen::Matrix3d gaussRot;
               // gaussRot.col(0) = ex;
               // gaussRot.col(1) = ey;
               // gaussRot.col(2) = ez;
               // Eigen::Matrix3d diagMat; diagMat.setIdentity();
               // diagMat(0,0) = sqrt(lx);diagMat(1,1) = sqrt(ly);diagMat(2,2) = sqrt(lz);
               // 
               // Eigen::Quaterniond quat(gaussRot);

               // 
               // marker.pose.orientation.x = quat.x();
               // marker.pose.orientation.y = quat.y();
               // marker.pose.orientation.z = quat.z();
               // marker.pose.orientation.w = quat.w();
                marker.scale.x = resolution;//FIXME use proper value here  //sqrt(lx)*3;
                marker.scale.y = resolution;                               //sqrt(ly)*3;

                
                if(cell->clusters[c].num_pts == 1) {
                    marker.scale.x = 0.1;
                    marker.scale.y = 0.1;
                    marker.scale.z = 0.1;
                } else {
                    marker.scale.z = sqrt(cell->clusters[c].cov(2,2))*cluster_sigma_factor;
                }
         
                if(cell->clusters[c].num_pts < min_cluster_points) {
                    marker.color.a = 0.1; //cell with few points are more transparent
                } else {
                    marker.color.a = 0.6;
                }

                if(cell->drivable) {
                    if (c == ground_idx || (ground_idx == -1 && c == 0)) {
                        marker.color.r = 0;
                        marker.color.g = 255;
                        marker.color.b = 0;
                    } else {
                        marker.color.r = 0;
                        marker.color.g = 0;
                        marker.color.b = 255;
                    }
                } else {
                    marker.color.r = 255;
                    marker.color.g = 0;
                    marker.color.b = 0;
                }
                    
                marker_array.markers.push_back(marker);
            }
        }
		
	}

    prev_size = idx_count;
    
	pub.publish(marker_array);

}
