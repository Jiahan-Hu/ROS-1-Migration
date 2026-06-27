#include <ground_seg.h>

void ground_seg::removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}


/*downsample the original point cloud*/
pcl::PointCloud<PointT>::ConstPtr ground_seg::downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) 
{
	if(!downsample_filter) 
	{
	  return cloud;
	}
	pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
	downsample_filter->setInputCloud(cloud);
	downsample_filter->filter(*filtered);
	filtered->header = cloud->header;
	return filtered;
}

void ground_seg::ransac_plane_fitting(pcl::PointCloud<PointT>::Ptr input_points, 
						  pcl::PointCloud<PointT>::Ptr &output_plane_points_, 
						  pcl::PointCloud<PointT>::Ptr &output_nonplane_points_)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr points_xyz(new pcl::PointCloud<pcl::PointXYZ>());

	for (int i = 0; i < input_points->size(); ++i)
	{
		pcl::PointXYZ point;
		point.x=input_points->points[i].x;
		point.y=input_points->points[i].y;
		point.z=input_points->points[i].z;
		points_xyz->points.push_back(point);

	}

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold(DistanceThreshold);
	seg.setInputCloud (points_xyz);
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0)
	{
	ROS_ERROR ("Could not estimate a planar model for the given dataset.\n");
	}

	std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                     << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

	std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;


	pcl::PointCloud<PointT>::Ptr cloud_ground_part(new pcl::PointCloud<PointT>());

	// for (const auto& idx: inliers->indices)
	// {
	// 	PointT point;
	// 	point.x=points_xyz->points[idx].x;
	// 	point.y=points_xyz->points[idx].y;
	// 	point.z=points_xyz->points[idx].z;
	// 	point.intensity=30;
	// 	output_plane_points_->points.push_back(point);
	// }

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>), 
										cloud_nonplane (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;	

    extract.setInputCloud (points_xyz);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_plane);

    extract.setNegative (true);
    extract.filter (*cloud_nonplane);
    // cloud_filtered.swap (cloud_f);
    if (!cloud_plane->empty())
    {
	    for (int i = 0; i < cloud_plane->size(); ++i)
	    {
	    	PointT p;
	    	p.x=cloud_plane->points[i].x;
	    	p.y=cloud_plane->points[i].y;
	    	p.z=cloud_plane->points[i].z;
	    	p.intensity=10;
	    	output_plane_points_->points.push_back(p);
	    }
    }

    if (!cloud_nonplane->empty())
    {
   		for (int i = 0; i < cloud_nonplane->size(); ++i)
	    {
	    	PointT p;
	    	p.x=cloud_nonplane->points[i].x;
	    	p.y=cloud_nonplane->points[i].y;
	    	p.z=cloud_nonplane->points[i].z;
	    	p.intensity=0;
	    	output_nonplane_points_->points.push_back(p);
	    }
    }


}

void ground_seg::combine_cloud(pcl::PointCloud<PointT>::Ptr &cloud_1, 
				   pcl::PointCloud<PointT>::Ptr cloud_2)
{
	if (!cloud_2->empty())
	{
		for (int i = 0; i < cloud_2->size(); ++i)
		{
			cloud_1->points.push_back(cloud_2->points[i]);
		}
	}
}

void ground_seg::ground_segmentation(pcl::PointCloud<PointT>::Ptr cloud, 
						 pcl::PointCloud<PointT>::Ptr &output_plane_points__,
						 pcl::PointCloud<PointT>::Ptr &output_nonplane_points__,sensor_msgs::PointCloud2 laserCloudMsg)
{
	pcl::PointCloud<PointT>::Ptr cropped_points(new pcl::PointCloud<PointT>()),
    							 remain_points(new pcl::PointCloud<PointT>());
 	pcl::PointCloud<PointT>::Ptr output_plane_points (new pcl::PointCloud<PointT>());
  	pcl::PointCloud<PointT>::Ptr output_nonplane_points (new pcl::PointCloud<PointT>());

 	double x0=0,x1=20,x2=30,x3=80,x4=-20,x5=-30,x6=-80;
	double height_0,height_1=0,height_2=0,offset=0.6;

	double height_inter=average_height;
	for (int i = 0; i < 5; ++i)
	{
	    for (int ii = 0; ii < cloud->size(); ++ii)
	    {
	    	PointT point;

	    	point.x=cloud->points[ii].x;
	    	point.y=cloud->points[ii].y;
	    	point.z=cloud->points[ii].z;
	    	point.intensity=cloud->points[ii].intensity;
	    	if (point.z>=height_inter-height_offset && point.z<=height_inter+height_offset && point.x>=x5 && point.x<=x2)
	    	{
	    		cropped_points->points.push_back(point);
	    	}else if(point.x>=x5 && point.x<=x2)
	    	{
	    		remain_points->points.push_back(point);
	    	}
	    }
		ransac_plane_fitting(cropped_points,output_plane_points,output_nonplane_points);
		double counter_=0;
		for (int j = 0; j < output_plane_points->size(); ++j)
		{
			PointT p=output_plane_points->points[j];
			height_0=(counter_*height_0+p.z)/(counter_+1);//for region 1
			counter_++;
		}
	    // std::cout<<"height_0 = "<<height_0<<std::endl;
	    // std::cout<<"height_inter = "<<height_inter<<std::endl;	
		if (abs(height_inter-height_0)<0.1  ||  i==5)
		{
			if (height_0<=average_height-height_offset*0.5  ||  height_0>=average_height+height_offset*0.5)
			{
				height_0=average_height;
			}
			break;
		}else
		{
			if (height_0<=average_height-height_offset*0.5  ||  height_0>=average_height+height_offset*0.5)
			{
				height_0=average_height;
			}
			height_inter=height_0;
			cropped_points->clear();
			output_plane_points->clear();
			output_nonplane_points->clear();
			remain_points->clear();
		}
		// std::cout<<"height_inter = "<<height_inter<<std::endl;
	}

	/*publishing the cropped points*/
	sensor_msgs::PointCloud2 cropped_points_;
    pcl::toROSMsg(*cropped_points, cropped_points_);
    cropped_points_.header.stamp = laserCloudMsg.header.stamp;
    cropped_points_.header.frame_id =laserCloudMsg.header.frame_id;
	points_cropped_pub.publish(cropped_points_);

	/**/

	// ransac_plane_fitting(cropped_points,output_plane_points,output_nonplane_points);


	double counter_1=0,counter_2=0;
	for (int i = 0; i < output_plane_points->size(); ++i)
	{
		PointT p=output_plane_points->points[i];
		
		if (p.x>=x5 && p.x<=x0)
		{
			height_1=(counter_1*height_1+p.z)/(counter_1+1);//for region 1
			counter_1++;
		}
		if (p.x>=x0 && p.x<=x2)
		{
			height_2=(counter_2*height_2+p.z)/(counter_2+1);//for region 2
			counter_2++;
		}
	}

    std::cout<<"counter_1 = "<<counter_1<<std::endl;	
    std::cout<<"counter_2 = "<<counter_2<<std::endl;	

    std::cout<<"height_1 = "<<height_1<<std::endl;	
    std::cout<<"height_2 = "<<height_2<<std::endl;	

    pcl::PointCloud<PointT>::Ptr region_1(new pcl::PointCloud<PointT>()),region_1_plane(new pcl::PointCloud<PointT>()),region_1_nonplane(new pcl::PointCloud<PointT>()),
								 region_2(new pcl::PointCloud<PointT>()),region_2_plane(new pcl::PointCloud<PointT>()),region_2_nonplane(new pcl::PointCloud<PointT>());

    for (int i = 0; i < cloud->size(); ++i)
    {
    	PointT point;

    	point.x=cloud->points[i].x;
    	point.y=cloud->points[i].y;
    	point.z=cloud->points[i].z;
    	point.intensity=cloud->points[i].intensity;
    	if (point.z>=height_1-offset && point.z<=height_1+offset &&  point.x>=x6  &&  point.x<=x5)//region 1
    	{
    		region_1->points.push_back(point);
    	}

    	if (point.z>=height_2-offset && point.z<=height_2+offset &&  point.x>=x2  &&  point.x<=x3) //region 2
    	{
    		region_2->points.push_back(point);
    	}
    }	

    std::cout<<"region_1 size = "<<region_1->size()<<std::endl;	
    std::cout<<"region_2 size = "<<region_2->size()<<std::endl;						 

    ransac_plane_fitting(region_1,region_1_plane,region_1_nonplane);
    ransac_plane_fitting(region_2,region_2_plane,region_2_nonplane);

    pcl::PointCloud<PointT>::Ptr region_1_(new pcl::PointCloud<PointT>()),region_1_plane_(new pcl::PointCloud<PointT>()),
    							 region_1_nonplane_(new pcl::PointCloud<PointT>()),region_1_remain_(new pcl::PointCloud<PointT>()),
							 	 region_2_(new pcl::PointCloud<PointT>()),region_2_plane_(new pcl::PointCloud<PointT>()),
							 	 region_2_nonplane_(new pcl::PointCloud<PointT>()),region_2_remain_(new pcl::PointCloud<PointT>());
 	

 	double height_1_=0,height_2_=0,offset_=0.6;
	counter_1=0,counter_2=0;
	for (int i = 0; i < region_1_plane->size(); ++i)
	{
		PointT p=region_1_plane->points[i];
		
		if (p.x>=x6 && p.x<=x5)
		{
			height_1_=(counter_1*height_1_+p.z)/(counter_1+1);//for region 1
			counter_1++;
		}
	}

	for (int i = 0; i < region_2_plane->size(); ++i)
	{
		PointT p=region_2_plane->points[i];
		if (p.x>=x2 && p.x<=x3)
		{
			height_2_=(counter_2*height_2_+p.z)/(counter_2+1);//for region 2
			counter_2++;
		}
	}

    std::cout<<"counter_1 = "<<counter_1<<std::endl;	
    std::cout<<"counter_2 = "<<counter_2<<std::endl;	

    std::cout<<"height_1_ = "<<height_1_<<std::endl;	
    std::cout<<"height_2_ = "<<height_2_<<std::endl;	

    for (int i = 0; i < cloud->size(); ++i)
    {
    	PointT point;

    	point.x=cloud->points[i].x;
    	point.y=cloud->points[i].y;
    	point.z=cloud->points[i].z;
    	point.intensity=cloud->points[i].intensity;
    	if (point.z>=height_1_-offset_ && point.z<=height_1_+offset_ &&  point.x>=x6  &&  point.x<=x5)//region 1
    	{
    		region_1_->points.push_back(point);
    	}else if (point.x>=x6  &&  point.x<=x5)
    	{
    		region_1_remain_->points.push_back(point);
    	}

    	if (point.z>=height_2_-offset_ && point.z<=height_2_+offset_ &&  point.x>=x2  &&  point.x<=x3) //region 2
    	{
    		region_2_->points.push_back(point);
    	}else if (point.x>=x2  &&  point.x<=x3)
    	{
    		region_2_remain_->points.push_back(point);
    	}
    }

    ransac_plane_fitting(region_1_,region_1_plane_,region_1_nonplane_);
    ransac_plane_fitting(region_2_,region_2_plane_,region_2_nonplane_);


    /////////////////////////////////////////////////////////combine the ground points together
    combine_cloud(output_plane_points,region_1_plane_);
    combine_cloud(output_plane_points,region_2_plane_);
	/////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////combine the non ground points together
    combine_cloud(output_nonplane_points,remain_points);
    combine_cloud(output_nonplane_points,region_1_remain_);
    combine_cloud(output_nonplane_points,region_1_nonplane_);
    combine_cloud(output_nonplane_points,region_2_remain_);
    combine_cloud(output_nonplane_points,region_2_nonplane_);
    /////////////////////////////////////////////////////////


	/*publishing the ground points*/
	if (!output_plane_points->empty())
	{
		sensor_msgs::PointCloud2 ground_points;
	    pcl::toROSMsg(*output_plane_points, ground_points);
	    ground_points.header.stamp = laserCloudMsg.header.stamp;
	    ground_points.header.frame_id =laserCloudMsg.header.frame_id;
		points_ground_pub.publish(ground_points);
	}
	/**/

	/*publishing the non-ground points*/
	if (!output_nonplane_points->empty())
	{
		sensor_msgs::PointCloud2 nonground_points;
	    pcl::toROSMsg(*output_nonplane_points, nonground_points);
	    nonground_points.header.stamp = laserCloudMsg.header.stamp;
	    nonground_points.header.frame_id =laserCloudMsg.header.frame_id;
		points_no_ground_pub.publish(nonground_points);
	}
	/**/

	output_plane_points__=output_plane_points;
	output_nonplane_points__=output_nonplane_points;
}

/*processing the pointcloud from velodyne*/
void ground_seg::velydyne_handler(sensor_msgs::PointCloud2 laserCloudMsg)
{


    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(laserCloudMsg, *cloud);

	pcl::PointCloud<PointT>::Ptr output_plane_points (new pcl::PointCloud<PointT>());
  	pcl::PointCloud<PointT>::Ptr output_nonplane_points (new pcl::PointCloud<PointT>());
    
    ground_segmentation(cloud,output_plane_points,output_nonplane_points,laserCloudMsg);

	// std::cout<<"output_plane_points size = "<<output_plane_points->size()<<std::endl;	
	// output_plane_points->clear();
	// std::cout<<"output_plane_points size = "<<output_plane_points->size()<<std::endl;	

	
}

