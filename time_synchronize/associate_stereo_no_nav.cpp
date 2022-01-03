//CXX 11 Required.
//This code is to produce a asscociate.txt
//association2 is used to symchrom the imu from lpms, bluefox matrix version 200w, blue sonar, and the realsense tracking camera. 
//synchronize imu, image and sonar depth
//format: time_stamp -> image_dir -> sonar_depth -> qw -> qx -> qy -> qz  -> ax -> ay -> az;
//lpnav: the yaw angle is in degree. 

#include <iostream>
#include <string>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <cmath>

using namespace std;
using namespace Eigen;


int main(int argc, char** argv){

  if (argc!=2)
  {
    
    cout<<"Usage: Need data_set directory."<<endl;
    return 1;
  }

  string file_dir(argv[1]);

  ifstream file_sonar(file_dir + (string) "/sonar_data/ping_sonar.txt");
  ifstream file_image(file_dir + (string) "/t265_data/images.txt");
  ifstream file_lpimu(file_dir + (string) "/lpimu_data/imu_data.txt");
  //ifstream file_lpnav(file_dir + (string) "/lpnav_data/lp_nav.txt");

  ofstream file_associate(file_dir + (string) "/associate_stereo_no_nav.txt", ios::trunc| ios::out);

  if (!file_lpimu| !file_sonar| !file_image)
  {
    cout<<"Cannot Open files. "<<endl;
    return 0;
  }

  vector<long> imu_time;
  
  // imu_pose (x, y, z), imu_accel (a_x, a_y, a_z)
  vector<Vector3d> imu_pose, imu_accel;
  
  // imu_quaternion (x, y, z, w)
  vector<Vector4d> imu_quaternion;


  string time_stamp;
  
  double x, y, z, a_x, a_y, a_z, q_x, q_y, q_z, q_w;
  
 cout<<"Start Reading StereoImages"<<endl;

 vector<long> stereo_times;
 vector<string> images_left;
 vector<string> images_right;
 string img_left;
 string img_right;
 bool odd = true;

 while(!file_image.eof()){

	 if(odd){
		 file_image >> time_stamp;
		 file_image >> img_left;
		 int n = img_left.length();
		 char char_array[n+1];
		 strcpy(char_array, img_left.c_str());
		 for(int i =0; i<3; i++){
			 char_array[n-i]=char_array[n-i-1];
		 }
		 char_array[n-3]='.';
		 string name = "";
		 for( int i =0; i<sizeof(char_array); i++){
			 name = name + char_array[i];
		 }

		 stereo_times.push_back(stol(time_stamp));
		 images_left.push_back(name);
		 odd = false;
	 }else{

		file_image >> time_stamp;
		file_image >> img_right;
		images_right.push_back(img_right);       
		odd = true;
	 }
 }

 cout<<"Start Reading LPIMU_FILE"<<endl;
 vector<Vector4d> lpimu_quaternion;
 vector<long> lpimu_time;
 vector<Vector3d> lpimu_accel;
 double lq_w, lq_x, lq_y, lq_z, la_x, la_y, la_z; 

 while(!file_lpimu.eof()){
   file_lpimu>>time_stamp>>lq_w>>lq_x>>lq_y>>lq_z>>la_x>>la_y>>la_z;
   lpimu_time.push_back(stol(time_stamp));
   lpimu_quaternion.push_back(Vector4d(lq_w, lq_x, lq_y, lq_z));
   lpimu_accel.push_back(Vector3d(la_x, la_y, la_z));
 }



 string depth;
 string dot1;
 string dot2;
 string confidence;
 vector<double> sonar_depth;
 vector<long> sonar_time;

 cout <<"Start Reading SONAR_FILE"<<endl;

 while(!file_sonar.eof()){

   file_sonar>>time_stamp>>depth>>confidence;


   
   try{
   sonar_time.push_back(stol(time_stamp));
   }catch(...)
   {
	   continue;
   }

   sonar_depth.push_back(stod(depth)/1000);
  }


 cout <<"Start Synchronization...."<<endl;

 cout <<"Image size is: "<<stereo_times.size()<<endl; 

 for (int i=50; i<(stereo_times.size()-1); i++)
 {

   cout << "Time Stamp: "<<stereo_times[i]<<endl;

   file_associate << stereo_times[i] <<" "<<images_left[i]<<" "<<images_right[i]<<" ";
   
   int sonar_diff=10000000;
   int id_sonar=i;

   for (int j=0; j<sonar_time.size(); j++)
   {

      int sonar_diff_prev=sonar_diff;
      sonar_diff = abs(stereo_times[i] - sonar_time[j]);

      if (sonar_diff < sonar_diff_prev)
        id_sonar = j;
   }
   //check sonar.txt last line
   //cout<<"Id of sonar depth is: "<<id_sonar<<endl;
   file_associate << sonar_depth[id_sonar]<<" ";

  
   int id_lpimu=i;
   int lpimu_diff = 10000000;
   for(int k=0; k <lpimu_time.size(); k++)
   {
     int lpimu_diff_prev=lpimu_diff;

     lpimu_diff = abs(stereo_times[i] - lpimu_time[k]);

     if (lpimu_diff < lpimu_diff_prev)

       id_lpimu = k;

   }


   file_associate << lpimu_quaternion[id_lpimu](0) <<" "<< lpimu_quaternion[id_lpimu](1)<<" "<< lpimu_quaternion[id_lpimu](2) <<" "<< lpimu_quaternion[id_lpimu](3)<<" "<<" "
    <<lpimu_accel[id_lpimu](0)<<" "<<lpimu_accel[id_lpimu](1)<<" "<<lpimu_accel[id_lpimu](2)<<endl;


 }

 cout << "Synchronization Finished. "<<endl;
 return 0;


}
