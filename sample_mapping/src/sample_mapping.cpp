#include "sample_mapping/sample_mapping.h"

using namespace std;

SampleMap::SampleMap(int xbins, int ybins, double res):sample_grid(new nav_msgs::OccupancyGrid)
    {
				
		size_x = xbins;
        size_y = ybins;
        resolution = res;
        fov_cone = M_PI/2;
        
        size_x_meters = size_x*resolution;
        size_y_meters = size_y*resolution;
        xmin_meters =  -1.0*size_x_meters/2; 
		ymin_meters =  -1.0*size_y_meters/2;
		
		sample_grid->header.frame_id = "/global";
        sample_grid->info.resolution = res;
        sample_grid->info.width = size_x;
        sample_grid->info.height = size_y;
        geometry_msgs::Pose offset;
        offset.position.x = -(resolution*size_x/2);
        offset.position.y = -(resolution*size_y/2);
        sample_grid->info.origin = offset;
        sample_grid->data.resize(size_x*size_y);
        
        //allocate data for the sample grid map
        data = (cell*)malloc(size_x*size_y*sizeof(cell));
        
        //init the opecv map
        cv_map = cv::Mat::zeros(size_x, size_y, CV_8UC1);
        pixel_prob_threshold = 0.95;
        
        initalizeSampleMap();
	}

int SampleMap::mapXYToLinear(double x, double y)  //converts global XY co-ordinates to a linear index for map cell
    {
		 int bindx = floor((x - xmin_meters) / resolution) ;
         int bindy = floor((y - ymin_meters) / resolution) ;
         
         if(bindx <0 || bindx >=size_x || bindy<0 || bindy>size_y) //invalid bin
         {
			return -1;
		 }
         else
         {
			return bindx+size_y*bindy; //linear index of bin 
		 }
	}
	
	int SampleMap::mapIJToXY(double i, double j, double& x, double& y)
	{
		x = resolution*i -(resolution*size_x/2);
		y = resolution*j -(resolution*size_y/2);
	}
	
	int SampleMap::mapIJToLinear(double i, double j) //maps 2 index in cells to linear index
	{
		return i*size_y + j; //linear index of bin 
	}
	
	int SampleMap::mapProbabilityToOccupancy(double _input_range_min, double _input_range_max, double _input_value_tobe_converted)
	{
		double _output_range_min = 0; 
        double _output_range_max = 127;
		double diffOutputRange = fabs((_output_range_max - _output_range_min));
		double diffInputRange = fabs((_input_range_max - _input_range_min));
		double convFactor = (diffOutputRange / diffInputRange);
		return floor((_output_range_min + (convFactor * (_input_value_tobe_converted - _input_range_min))));
	}
   
   
	
	void SampleMap::initalizeSampleMap()
	{
		for(int i=0; i<size_x; i++)
		{
			for(int j=0; j<size_y; j++)
			{
				int lidx = mapIJToLinear(i,j);
				data[lidx].probability = 0;
				sample_grid->data[lidx] = -1; 
				
			}
		}
	}
	
	
	
	void SampleMap::addSampleToMap(double x, double y, SampleGaussian sample) //adds a sample at global x y to the map with a mean prob and cov
	{
		
		//figure out the update window size based on 3 sigma bound
		int window_size = floor(sample.getBound()/resolution);
		//now iterate over the windowsize in the map and update the probabilities
		
		for(int i = -window_size; i<=window_size; i++)
		{
			for(int j = -window_size; j<=window_size; j++)
			{
				double xloc = x+i*resolution;
				double yloc = y+j*resolution;
				int lidx = mapXYToLinear(xloc,yloc);
				
				//std::cout<<xloc<<","<<yloc<<std::endl;
				
				if(lidx>0) //valid cell
				{
						double cell_prob = sample.getProbabilityValue(i*resolution,j*resolution);
						//std::cout<<cell_prob<<std::endl;
						//log odds update
						data[lidx].probability+=cell_prob;
						
						if(data[lidx].probability>integration_parameters.prob_max)
							data[lidx].probability = integration_parameters.prob_max;
						
						int	occProb = mapProbabilityToOccupancy(integration_parameters.prob_min, integration_parameters.prob_max, data[lidx].probability );
						//std::cout<<occProb<<std::endl;
						sample_grid->data[lidx] = occProb; 
				}
				
			}
		}
			
	}
	
	void SampleMap::removeConeFromMap(double x, double y, double theta, double dec_value)
	{
		//first find the cone's vector
		double rmax = integration_parameters.rmax;
		double rmin = integration_parameters.rmin;
		
		double cx = cos(theta)*rmax;
		double cy = sin(theta)*rmax;
		double cmag = sqrt(cx*cx + cy*cy);
		cx = cx/cmag; cy = cy/cmag; //cone unit vector
		
		 
		int window_size = floor(rmax/resolution);
		//now iterate over the windowsize in the map and update the probabilities
		
		for(int i = -window_size; i<=window_size; i++)
		{
			for(int j = -window_size; j<=window_size; j++)
			{
				double xloc = x+i*resolution;
				double yloc = y+j*resolution;
				int lidx = mapXYToLinear(xloc,yloc);
				
				if(lidx>0) //valid cell
				{
				
					double qx = (xloc-x);
					double qy = (yloc-y);
					double qpt_dist = sqrt(qx*qx + qy*qy);
					//compute unit vector
					qx = qx / qpt_dist; qy = qy/qpt_dist;
					
					//dot product
					double cosinus = cx*qx + cy*qy;
					double q_ang = acos(cosinus);
					
					//std::cout<<cosinus<<std::endl;
					
					//now we have enouggh to see if the point is in the cone
					if( qpt_dist > rmin &&  qpt_dist<rmax &&  fabs(q_ang) < (fov_cone/2.0)) //inside the cone
					{
						
						data[lidx].probability-=dec_value;
						
						if(data[lidx].probability<integration_parameters.prob_min)
							data[lidx].probability = integration_parameters.prob_min;
						
						int	occProb = mapProbabilityToOccupancy(integration_parameters.prob_min, integration_parameters.prob_max, data[lidx].probability );
						
						sample_grid->data[lidx] = occProb; 
						
					}
							
				}
			}
		}
		
	}
	
	void SampleMap::copyMaptoCV()
	{
		
		for(int i=0; i< size_x; i++)
		{
			for(int j = 0; j<size_y; j++)
			{
				
				int lidx = mapIJToLinear(i,j);
				double pixel_prob = data[lidx].probability/integration_parameters.prob_max; //normalize 0 -1
				if(pixel_prob>pixel_prob_threshold)
					cv_map.at<uint8_t>(i,j) = 255;
				else
					cv_map.at<uint8_t>(i,j) = 0;
				
			}
		}
	}
	
	void SampleMap::processMapCV()
	{
		sample_points.clear(); //clear current samples;
		vector<vector<cv::Point> > contours;
		vector<cv::Vec4i> hierarchy;
		cv::Mat contour_mat = cv_map;
		cv::findContours( contour_mat, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		
		//get the contour centroids
		
		for (int i=0; i<contours.size(); i++)
		{
			cv::Moments contour_moment;
						vector<cv::Point> cur_contour = contours[i];
			int num_contour_points = cur_contour.size();
			cv::Point avg;
			for(int j=0; j<num_contour_points; j++)
			{
					avg+=cur_contour[j];
			}
			avg = avg*(1.0/((double)num_contour_points));
			
			//map these back to XY global
			
			double gx = 0; double gy = 0;
			mapIJToXY(avg.x, avg.y, gx, gy);
			cv::Point2d global_sample_location(gx,gy);
			//cout<<global_sample_location<<endl;
			sample_points.push_back(global_sample_location);
			
		}
		
	}
