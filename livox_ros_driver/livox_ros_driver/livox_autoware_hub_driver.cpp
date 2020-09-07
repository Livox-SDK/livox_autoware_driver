//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "lddc.h"

using namespace std;
bool firstFlag = true;
int firstID = 10;
int hub_lidar_num;
ros::Time firstTime;
bool sync_init = false;
ros::Publisher pubFullLaserCloud;
std::vector<Eigen::Matrix4f> extrinsicMatrix;

bool loadExtrinsic(const std::string &filename, std::vector<Eigen::Matrix4f>& mat)
{
  FILE *fp=fopen(filename.c_str(),"r");
  int nNum=0;
  float roll,pitch,yaw,x,y,z;
  Eigen::Matrix3f rotation_matrix;

  while(!feof(fp))
  {
    if(nNum >= int(mat.size())){
      break;
    }
    if(fscanf(fp,"%f,%f,%f,%f,%f,%f,\n",&roll,&pitch,&yaw,&x,&y,&z) == 0){
      return false;
    }

    rotation_matrix = Eigen::AngleAxisf(yaw,Eigen::Vector3f::UnitZ()) *
                      Eigen::AngleAxisf(pitch,Eigen::Vector3f::UnitY()) *
                      Eigen::AngleAxisf(roll,Eigen::Vector3f::UnitX());

    mat[nNum] = Eigen::Matrix4f::Identity();
    mat[nNum].topLeftCorner(3,3) = rotation_matrix;
    mat[nNum].topRightCorner(3,1) = Eigen::Vector3f(x,y,z);
    nNum++;
  }
  fclose(fp);
  return true;
}

void pubPointCloud2(const pcl::PointCloud<pcl::PointXYZI>::Ptr& ptr){
  sensor_msgs::PointCloud2 laserCloudMsg;
  pcl::toROSMsg(*ptr, laserCloudMsg);
  laserCloudMsg.header.stamp = ros::Time::now();
  laserCloudMsg.header.frame_id = "/livox_frame";
  pubFullLaserCloud.publish(laserCloudMsg);
  cout << "---------------------" << endl;
}

void hubCallBack(const livox_ros_driver::CustomMsgConstPtr &msg){
  static pcl::PointCloud<pcl::PointXYZI>::Ptr all_in_one_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  int lidar_id = int(msg->lidar_id) / 3;
  if(lidar_id >= hub_lidar_num) return;
  pcl::PointXYZI tmp;
  Eigen::Vector3f eig_p;
  if(abs(msg->header.stamp.toSec()-firstTime.toSec()) > 0.09 || lidar_id == firstID)firstFlag = true;
  if(firstFlag)
  {
    if(!all_in_one_ptr->points.empty())
    {
      cout << "all_in_one_ptr->points.size()=" << all_in_one_ptr->points.size() << endl;
      pubPointCloud2(all_in_one_ptr);
      firstFlag = true;
      all_in_one_ptr->clear();
    }


    firstFlag = false;
    firstID = lidar_id;
    firstTime = msg->header.stamp;
    for(const auto & point : msg->points){
      //if(point.x < 1.0) continue;
      eig_p.x() = point.x;
      eig_p.y() = point.y;
      eig_p.z() = point.z;
      eig_p = (extrinsicMatrix[lidar_id].topLeftCorner(3,3) * eig_p
              + extrinsicMatrix[lidar_id].topRightCorner(3,1)).eval();
      tmp.x = eig_p.x();
      tmp.y = eig_p.y();
      tmp.z = eig_p.z();
      tmp.intensity = point.reflectivity;
      all_in_one_ptr->push_back(tmp);
    }
    cout << "first lidar id=" << firstID << endl;
    cout << "lidar ID=" << lidar_id << "    timestamp=" << msg->header.stamp << endl;
  }


  if(abs(msg->header.stamp.toSec() - firstTime.toSec()) <= 0.08 && lidar_id != firstID)
  {
    for(const auto & point : msg->points){
      //if(point.x < 1.0) continue;
      eig_p.x() = point.x;
      eig_p.y() = point.y;
      eig_p.z() = point.z;
      eig_p = (extrinsicMatrix[lidar_id].topLeftCorner(3,3) * eig_p
              + extrinsicMatrix[lidar_id].topRightCorner(3,1)).eval();
      tmp.x = eig_p.x();
      tmp.y = eig_p.y();
      tmp.z = eig_p.z();
      tmp.intensity = point.reflectivity;
      all_in_one_ptr->push_back(tmp);
    }
    cout << "not first flag" << endl;
    cout << "lidar ID=" << lidar_id << "    timestamp=" << msg->header.stamp << endl;
  }

    
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "livox_autoware_hub_node");
  ros::NodeHandle livox_node("~");

  firstFlag = true;
  firstID = 10;
  /** Init defualt system parameter */
  int xfer_format = 0;
   livox_node.getParam("/xfer_format", xfer_format);
   if(xfer_format != livox_ros::kLivoxCustomMsg){
     ROS_WARN("NOTE: Request 'xfer_format = 1' !");
     return 1;
   }

   if(!livox_node.getParam("/hub_lidar_num", hub_lidar_num)){
     ROS_WARN("NOTE: Request 'hub_lidar_num' param!");
     return 1;
   }

   extrinsicMatrix.resize(hub_lidar_num, Eigen::Matrix4f::Identity());
  //extrinsicMatrix.resize(hub_lidar_num, Eigen::Matrix4f::Identity());

  std::string extrinsic_path;

   if(!livox_node.getParam("/extrinsic_file", extrinsic_path)){
     ROS_WARN("NOTE: Request 'extrinsic_file' param!");
     return 1;
   }

  if(!loadExtrinsic(extrinsic_path, extrinsicMatrix)){
    ROS_WARN("ERROR Load Extrinsic Matrix!");
    return 1;
  }

  std::cout<<"Extrinsic Matrixs:"<<std::endl;
  for(int i=0; i<hub_lidar_num; ++i){
    std::cout<<extrinsicMatrix[i]<<std::endl<<"----------------------"<<std::endl;
  }


  ros::Subscriber subFullCloud = livox_node.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 100, hubCallBack);
  pubFullLaserCloud = livox_node.advertise<sensor_msgs::PointCloud2>("/points_raw", 100);

  ros::spin();

  return 0;
}

