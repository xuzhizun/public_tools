//CXX 11 Required.
//This code is to produce a asscociate.txt 
//synchronize imu, image and sonar depth
//format: time_stamp -> image_dir -> sonar_depth -> qx -> qy -> qz -> qw -> ax -> ay -> az -> px -> py -> pz;

#include <iostream>
#include <string>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>

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
  ofstream file_associate(file_dir + (string) "/associate.txt", ios::trunc| ios::out);

  if (!file_imu | !file_sonar| !file_image)
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

   
    imu_time.push_back(stol(time_stamp));

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
   cout<<depth<<endl;
   sonar_depth.push_back(stod(depth)/1000);
  }

 
 cout <<"Start Synchronization...."<<endl;

 cout <<"Image size is: "<<image_time.size()<<endl; 

 for (int i=0; i<image_time.size(); i++)
 {

   cout << "Time Stamp: "<<image_time[i]<<endl;

   file_associate << image_time[i] <<" "<<image_path[i]<<" ";
   
   int sonar_diff=1000000;
   int id_sonar=i;

   for (int j=0; j<sonar_time.size(); j++)
   {

      int sonar_diff_prev=sonar_diff;

      sonar_diff = abs(image_time[i] - sonar_time[j]);

      if (sonar_diff < sonar_diff_prev)

        id_sonar = j;

   }

   file_associate << sonar_depth[id_sonar]<<" ";

   int id_imu=i;
   int imu_diff = 10000000;
   for(int k=0; k < imu_time.size(); k++)
   {
     int imu_diff_prev=imu_diff;

     imu_diff = abs(image_time[i] - imu_time[k]);

     if (imu_diff < imu_diff_prev)

       id_imu = k;

   }

   file_associate << imu_quaternion[id_imu](0) <<" "<< imu_quaternion[id_imu](1)<<" "<< imu_quaternion[id_imu](2) <<" "<< imu_quaternion[id_imu](3)<<" "
    <<imu_accel[id_imu](0)<<" "<<imu_accel[id_imu](1)<<" "<<imu_accel[id_imu](2)<<" "<< imu_pose[id_imu](0) <<" "<<imu_pose[id_imu](1) <<" "<<imu_pose[id_imu](2)<<endl;


 }

 cout << "Synchronization Finished. "<<endl;
 return 1;


}
