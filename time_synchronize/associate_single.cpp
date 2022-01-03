//CXX 11 Required.
//This code is to produce a asscociate.txt
//association2 is used to symchrom the imu from lpms, bluefox matrix version 200w, blue sonar, and the realsense tracking camera. 
//synchronize imu, image and sonar depth
//format: time_stamp -> image_dir -> sonar_depth -> qw -> qx -> qy -> qz ->yaw(in radian) -> ax -> ay -> az;
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

  ifstream file_imu(file_dir + (string) "/t265_data/imu.txt" );
  ifstream file_sonar(file_dir + (string) "/sonar_data/ping_sonar.txt");
  ifstream file_image(file_dir + (string) "/mv_images/mv_time_images.txt");
  ifstream file_lpimu(file_dir + (string) "/lpimu_data/imu_data.txt");
  ifstream file_lpnav(file_dir + (string) "/lpnav_data/lp_nav.txt");

  ofstream file_associate(file_dir + (string) "/monocular_associate.txt", ios::trunc| ios::out);

  if (!file_imu | !file_sonar| !file_image| !file_lpimu)
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
  
  cout<<"Start Reading IMU_FILE"<<endl;

  while(!file_imu.eof()){
    
    file_imu>>time_stamp>>x>>y>>z>>q_x>>q_y>>q_z>>q_w>>a_x>>a_y>>a_z;

    imu_pose.push_back(Vector3d(x,y,z));

    imu_accel.push_back(Vector3d(a_x, a_y, a_z));

    imu_quaternion.push_back(Vector4d(q_x, q_y, q_z, q_w));
  
    //cout<<"imu string: "<<time_stamp<<endl; 
    imu_time.push_back(stol(time_stamp));

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


  string path;
  
  vector<string> image_path;
  
  vector<long> image_time;

  cout<<"Start Reading IMAGE_FILE"<<endl;

  while(!file_image.eof()){
    
    file_image>>time_stamp>>path;
    image_time.push_back(stol(time_stamp));
    image_path.push_back(path);

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

   sonar_time.push_back(stol(time_stamp));
   sonar_depth.push_back(stod(depth)/1000);
  }

 double angle_de;
 vector<double> lpnav_angle;
 vector<long> lpnav_time;

 cout <<"Start Reading LPNAV_FILE"<<endl;

 while(!file_lpnav.eof()){

   file_lpnav>>time_stamp>>angle_de;

   lpnav_time.push_back(stol(time_stamp));
   lpnav_angle.push_back(angle_de/180.0*M_PI);
  }

 cout <<"Start Synchronization...."<<endl;

 cout <<"Image size is: "<<image_time.size()<<endl; 

 for (int i=50; i<(image_time.size()-1); i++)
 {

   cout << "Time Stamp: "<<image_time[i]<<endl;

   file_associate << image_time[i] <<" "<<image_path[i]<<" ";
   
   int sonar_diff=10000000;
   int id_sonar=i;

   for (int j=0; j<sonar_time.size(); j++)
   {

      int sonar_diff_prev=sonar_diff;
      sonar_diff = abs(image_time[i] - sonar_time[j]);

      if (sonar_diff < sonar_diff_prev)
        id_sonar = j;
   }
   //check sonar.txt last line
   //cout<<"Id of sonar depth is: "<<id_sonar<<endl;
   file_associate << sonar_depth[id_sonar]<<" ";

   /*
   int id_imu=i;
   int imu_diff = 10000000;
   for(int k=0; k <imu_time.size(); k++)
   {
     int imu_diff_prev=imu_diff;

     imu_diff = abs(image_time[i] - imu_time[k]);

     if (lpimu_diff < imu_diff_prev)

       id_imu = k;

   }

   file_associate << imu_quaternion[id_imu](0) <<" "<< imu_quaternion[id_imu](1)<<" "<< imu_quaternion[id_imu](2) <<" "<< imu_quaternion[id_imu](3)<<" "
    <<imu_accel[id_imu](0)<<" "<<imu_accel[id_imu](1)<<" "<<imu_accel[id_imu](2)<<" "<< imu_pose[id_imu](0) <<" "<<imu_pose[id_imu](1) <<" "<<imu_pose[id_imu](2)<<endl;
*/
  
   int id_lpimu=i;
   int lpimu_diff = 10000000;
   for(int k=0; k <lpimu_time.size(); k++)
   {
     int lpimu_diff_prev=lpimu_diff;

     lpimu_diff = abs(image_time[i] - lpimu_time[k]);

     if (lpimu_diff < lpimu_diff_prev)

       id_lpimu = k;

   }

   int id_lpnav=i;
   int lpnav_diff = 10000000;
   for(int k=0; k <lpimu_time.size(); k++)
   {
     int lpnav_diff_prev=lpnav_diff;

     lpnav_diff = abs(image_time[i] - lpnav_time[k]);

     if (lpnav_diff < lpnav_diff_prev)

       id_lpnav = k;

   }

   file_associate << lpimu_quaternion[id_lpimu](0) <<" "<< lpimu_quaternion[id_lpimu](1)<<" "<< lpimu_quaternion[id_lpimu](2) <<" "<< lpimu_quaternion[id_lpimu](3)<<" "<<lpnav_angle[id_lpnav]<<" "
    <<lpimu_accel[id_lpimu](0)<<" "<<lpimu_accel[id_lpimu](1)<<" "<<lpimu_accel[id_lpimu](2)<<endl;


 }

 cout << "Synchronization Finished. "<<endl;
 return 0;


}
