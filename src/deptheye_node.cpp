/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */
#include "DepthEyeInterface.h"
 
#include <iomanip>
#include <fstream>
#include <ostream>

#include <thread>            
#include <mutex>             
#include <condition_variable>
#include <queue>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
using namespace Voxel;
using namespace std;

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

//char rawFrameQueue[42496 * 2];
//char XYZpointCloutQueue[4800 * sizeof(float) * 4];

class DepthEyeNode
{
    public:
        DepthEyeNode()
        {
            nh_private = ros::NodeHandle("~");
            pub_pointcloud = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("points", 1); // sensor_msgs::PointCloud2
            pub_depthimage = nh.advertise<sensor_msgs::Image>("image_raw", 1);
            nh_private.param<std::string>("frame", this->frame_id, "base_link");
            
            Voxel::logger.setDefaultLogLevel(LOG_ERROR);
            depthEyeSys.connect();
            //depthEyeSys.enableFilterHDR();
            //depthEyeSys.enableFilterFlyingPixel(500);
            // for 10fps long range option
            depthEyeSys.setMode(PointCloud::PRICISTION);
            // for 30fps  short range option
            //depthEyeSys.setMode(PointCloud::STANDARD);

            depthEyeSys.registerPointCloudCallback([this](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) { return this->pointcloudCallback(dc, frame, c); }); 
            depthEyeSys.registerDepthCallback([this](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) { return this->depthCallback(dc, frame, c); }); 
            depthEyeSys.start();
        }
        
        ~DepthEyeNode() 
        {
            this->depthEyeSys.disconnect();
            this->depthEyeSys.stop();
        }
        
        void depthCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) 
        {
            //cout << "Frame type" << c;
            const DepthFrame *d = dynamic_cast<const DepthFrame *>(&frame);
            
            if(!d)
            {
                std::cout << "Null frame captured? or not of type DepthFrame" << std::endl;
                return;
            }
            //(char *)d->depth.data() depth is Vector<float>
            
            PointCloud::FrameSize fs =  this->depthEyeSys.getRevolution();
            sensor_msgs::Image im;
            // http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html
            // encoding const std::string TYPE_32FC1="32FC1";
            im.header.frame_id = this->frame_id;
            im.height = fs.height;
            im.width = fs.width;
            im.encoding = "32FC1";
            im.is_bigendian = false;
            im.step = 4*fs.width;
            im.data.resize(fs.height*fs.width*sizeof(float));
            memcpy(&(im.data[0]), (char*) d->depth.data(), fs.height*fs.width*sizeof(float)); // who needs memory safety anyway?
            pub_depthimage.publish(im);
        }

        void pointcloudCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) 
        {
            const XYZIPointCloudFrame *d = dynamic_cast<const XYZIPointCloudFrame *>(&frame);
            
            if(!d)
            {
                std::cout << "Null frame captured? or not of type XYZIPointCloudFrame" << std::endl;
                return;
            }
            // uncomment these and temp buffer as defined at top of file to print to console
            //memcpy((char *)XYZpointCloutQueue, (char *)d->points.data(), sizeof(IntensityPoint)*d->points.size());	
            //printOutFrameInfo((float*)XYZpointCloutQueue);
            
            PointCloud::FrameSize fs =  this->depthEyeSys.getRevolution();
            pcl::PointCloud<pcl::PointXYZI> cloud;
            cloud.points.resize(fs.height*fs.width);
            cloud.header.frame_id = this->frame_id;
            //memcpy(&(cloud.points[0]), (char*)d->points.data(), fs.height*fs.width*sizeof(float)*4);//   &(cloudPCLptr2->points[0]), zividCloud.dataPtr(), nbrOfBytes);
            
            pcl::PointXYZI p;
            for(int y=0; y<fs.height; ++y) {
                for(int x=0; x<fs.width; ++x) {
                    int curr_idx = (y*80+x)*4;
                    //float depth = xyziPointFrame[curr_idx];
                    IntensityPoint ip = d->points[curr_idx];
                    p.x = ip.x;
                    p.y = ip.y;
                    p.z = ip.z;
                    p.intensity = ip.i;
                        
                    cloud.push_back(p);
                }
            }
            pub_pointcloud.publish(cloud);
        }
        
        int printOutFrameInfo(float *xyziPointFrame){
	 
            printf("%.2f\t%.2f\t%.2f\t%.2f\n", xyziPointFrame[6480], xyziPointFrame[6481], xyziPointFrame[6482], xyziPointFrame[6483]);
     
        	char buffer[(80+1)*(60/2)+1];
            char * out = buffer;
        	const uint16_t one_meter = static_cast<uint16_t>(1.0f);
            uint16_t coverage[80] = {};
           	PointCloud::FrameSize fs =  this->depthEyeSys.getRevolution();
        	for(int y=0; y<fs.height; ++y)
            {
                for(int x=0; x<fs.width; ++x)
                {
                    float depth = xyziPointFrame[y*80*4+4*x+2];
                    if(depth > 0 && depth < 1.0f)
                    {
                    	 coverage[x] += static_cast<uint16_t>(depth*10);//depth/0.1;
                    }else
                    	 coverage[x] = 0;
            	
                }

                if(y%2 == 1){
                    for(uint16_t & c : coverage)
                    {
                        *out++ =  " .,*nM#N@mn"[c/2];
                        c = 0;
                    }
                	*out++ = '\n';
            	}
            }
            *out++ = 0;
            printf("\n%s", buffer);
        	return 0;
        }

    private:
        ros::NodeHandle nh;
        ros::NodeHandle nh_private;
        ros::Publisher pub_pointcloud;
        ros::Publisher pub_depthimage;
        
        std::string frame_id;
        
        PointCloud::DepthEyeSystem depthEyeSys;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "deptheye_node");
    DepthEyeNode den = DepthEyeNode();
    ros::spin();
    return 0;
}

