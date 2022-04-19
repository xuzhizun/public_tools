#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <chrono>
using namespace std;


#include <CameraApi.h>
#include <stdio.h>

unsigned char*          g_pRgbBuffer;
int                     hCamera; 
tSdkFrameHead           sFrameInfo; 
BYTE*                   pbyBuffer; 

bool cameraInitConfig()
{
      int                     iCameraCounts = 1; 
      int                     iStatus=-1; 
      tSdkCameraDevInfo       tCameraEnumList; 
      tSdkCameraCapbility     tCapability;       
      int                     channel=3; 
   
      CameraSdkInit(1); 
   
      iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts); 
      std::cout<<"state SDK: "<<iStatus<<'\n'; 
      std::cout<<"detected cameras: "<<iCameraCounts<<'\n'; 

      if(iCameraCounts==0){ 
          return 0; 
      } 
   
      iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera); 
           std::cout<<"Camera Init state: "<<iStatus<<'\n'; 

      if(iStatus!=CAMERA_STATUS_SUCCESS){ 
	  std::cout<<"Open Camera Failed !\n";
          return 0; 
      } 
   
      CameraGetCapability(hCamera,&tCapability); 
   
  
      g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3); 

      CameraPlay(hCamera); 
   
   
      //if(tCapability.sIspCapacity.bMonoSensor){ 
      //   channel=1; 
          //CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8); 
      //}else{ 
       //   channel=3; 
          //CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8); 
      //} 

       //Gray image
       CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8); 
   
          //Manual Exposure mode  
       CameraSetAeState(hCamera, 0); 
          //set exposure time 30um 
       CameraSetExposureTime(hCamera, 10*1000); 

 	return true;
}

int trigger = 0;
 
void my_thread()
{
	if(cin.get())
		trigger = 1;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "single_node");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/image_raw", 1);	
	int frameCount = 0;
	thread t1(my_thread);
	if(cameraInitConfig())
	{
		ROS_INFO("The MindVision Camera Initialization Done!\n");
	        t1.detach();	
		while(nh.ok()){
			auto start = chrono::steady_clock::now();
			if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
                  	{
			      CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);
                      
			      cv::Mat matImage(sFrameInfo.iHeight,
						  sFrameInfo.iWidth,
						  sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
						  g_pRgbBuffer
						  );

				 sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), 
										sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? "mono8" : "bgr8", 
										matImage).toImageMsg();
  
				 pub.publish(msg);
  
				 CameraReleaseImageBuffer(hCamera,pbyBuffer);
				 frameCount++;
				 if(trigger) break;
 
			 }
			auto end = chrono::steady_clock::now();
			if( frameCount == 150)
			{
				frameCount = 0;
				double elapsed_time = static_cast<double>(chrono::duration_cast<chrono::milliseconds>(end - start).count());
				ROS_INFO("Image Size: %d x %d, Frame Rate: %f\n", sFrameInfo.iHeight, sFrameInfo.iWidth, 1000.0/elapsed_time);
			}
		}

		 CameraUnInit(hCamera);
		 free(g_pRgbBuffer);
	}

	 return 0;
}

		
	
