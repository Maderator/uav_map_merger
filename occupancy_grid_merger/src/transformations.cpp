

#include "transformations.h"
  
namespace merger2d {

/* UAV FRAME OF REFERENCE POSE TRANSFORMATION //{ */


  /* relMapPoseToGlobalUAVPose //{ */
  geometry_msgs::PosePtr recUAVRelToCurUAV(geometry_msgs::PoseConstPtr rel_map_p, geometry_msgs::PoseConstPtr loc_cur_p, geometry_msgs::PoseConstPtr loc_rec_p){
    geometry_msgs::PosePtr inv_lcp = invertPose(loc_cur_p);
    return compoundPose(inv_lcp, compoundPose(rel_map_p, loc_rec_p));
  }
  //}

  /* UAVPoseLocalToGlobal //{ */
  geometry_msgs::PosePtr poseUAVLocalToGlobal(geometry_msgs::PoseConstPtr rel_map_p, geometry_msgs::PoseConstPtr loc_rec_p) {
    return compoundPose(rel_map_p, loc_rec_p);
  }
  //}

//}

  /* compoundPose() //{ */
  geometry_msgs::PosePtr compoundPose(const geometry_msgs::PoseConstPtr p1, const geometry_msgs::PoseConstPtr p2){
  
    geometry_msgs::PosePtr p = boost::make_shared<geometry_msgs::Pose>();
    // initialized values to a nan!!! copy orientation
    double p1_yaw = getYaw(p1->orientation);
    double p2_yaw = getYaw(p2->orientation);

    p->position.x = (p2->position.x)*cos(p1_yaw) - (p2->position.y)*sin(p1_yaw) + (p1->position.x);
    p->position.y = (p2->position.x)*sin(p1_yaw) + (p2->position.y)*cos(p1_yaw) + p1->position.y;
    p->position.z = p1->position.z;

    p->orientation = p1->orientation;
    setYaw(p1_yaw + p2_yaw ,p->orientation);
  
    return p;
  }
  
  geometry_msgs::PosePtr compoundPose(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2){
    geometry_msgs::PosePtr p = boost::make_shared<geometry_msgs::Pose>();
    // initialized values to a nan!!! copy orientation
    double p1_yaw = getYaw(p1.orientation);
    double p2_yaw = getYaw(p2.orientation);

    p->position.x = (p2.position.x)*cos(p1_yaw) - (p2.position.y)*sin(p1_yaw) + (p1.position.x);
    p->position.y = (p2.position.x)*sin(p1_yaw) + (p2.position.y)*cos(p1_yaw) + p1.position.y;
    p->position.z = p1.position.z;

    p->orientation = p1.orientation;
    setYaw(p1_yaw + p2_yaw ,p->orientation);
  
    return p;
  }
  
  geometry_msgs::PosePtr compoundPose(const geometry_msgs::PoseConstPtr p1, const geometry_msgs::Pose& p2){
    geometry_msgs::PosePtr p = boost::make_shared<geometry_msgs::Pose>();
    // initialized values to a nan!!! copy orientation
    double p1_yaw = getYaw(p1->orientation);
    double p2_yaw = getYaw(p2.orientation);

    p->position.x = (p2.position.x)*cos(p1_yaw) - (p2.position.y)*sin(p1_yaw) + (p1->position.x);
    p->position.y = (p2.position.x)*sin(p1_yaw) + (p2.position.y)*cos(p1_yaw) + p1->position.y;
    p->position.z = p1->position.z;

    p->orientation = p1->orientation;
    setYaw(p1_yaw + p2_yaw ,p->orientation);
  
    return p;
  }
  //}
  
  /* transformPointXY() //{ */
  geometry_msgs::PointPtr transformPointXY(const geometry_msgs::PoseConstPtr pose, const geometry_msgs::PointConstPtr point){
    double yaw = getYaw(pose->orientation);
    geometry_msgs::PointPtr p = boost::make_shared<geometry_msgs::Point>();
    p->x = point->x*cos(yaw) - point->y*sin(yaw) + pose->position.x;
    p->y = point->x*sin(yaw) + point->y*cos(yaw) + pose->position.y;
    p->z = point->z;
  
    return p;
  }

  geometry_msgs::PointPtr transformPointXY(const geometry_msgs::PoseConstPtr pose, const Cell2DConstPtr cell){
    double yaw = getYaw(pose->orientation);
    geometry_msgs::PointPtr p = boost::make_shared<geometry_msgs::Point>();
    p->x = cell->col*cos(yaw) - cell->row*sin(yaw) + pose->position.x;
    p->y = cell->col*sin(yaw) + cell->row*cos(yaw) + pose->position.y;
    p->z = 0;
  
    if(isnan(p->x)){
      ROS_ERROR("transformations.cpp transformPointXY error... x:%f, y:%f, yaw:%f, cell x:%d y:%d, poseX:%f, poseY:%f", p->x, p->y, yaw, cell->col, cell->row, pose->position.x, pose->position.y);
    }

    return p;
  }
  //}
  
  /* invertPose() //{ */
  geometry_msgs::PosePtr invertPose(const geometry_msgs::PoseConstPtr pose){
    double yaw = getYaw(pose->orientation);
  
    geometry_msgs::PosePtr p = boost::make_shared<geometry_msgs::Pose>();
    p->position.x = -pose->position.x *cos(yaw) - pose->position.y*sin(yaw);
    p->position.y = pose->position.x*sin(yaw) - pose->position.y*cos(yaw);
    yaw = -yaw;

    p->orientation = pose->orientation;
    setYaw(yaw, p->orientation);
    return p;
  }

  geometry_msgs::PosePtr invertPose(const geometry_msgs::Pose& pose) {
    double yaw = getYaw(pose.orientation);
  
    geometry_msgs::PosePtr p = boost::make_shared<geometry_msgs::Pose>();
    p->position.x = -pose.position.x *cos(yaw) - pose.position.y*sin(yaw);
    p->position.y = pose.position.x*sin(yaw) - pose.position.y*cos(yaw);
    yaw = -yaw;

    p->orientation = pose.orientation;
    setYaw(yaw, p->orientation);
    return p;
  }
  //}
  
  /* transformCvImageByPose() //{ */
  void transformCvImageByPose(cv_bridge::CvImagePtr img_map, const geometry_msgs::PoseConstPtr pose, const geometry_msgs::Pose& origin, bool gui){

    //ros::Time begin_time = ros::Time::now();
    cv::Mat img = img_map->image; // does not copy mat
    cv::Mat resized_img;

    CV_Assert(img.depth() == CV_8U);
    CV_Assert(img.channels() == 1);

    if (gui) {
      cv::resize(img, resized_img, cv::Size(300,300)); 
      cv::imshow("Image before transformation", resized_img);
    }
    
    double yaw = getYaw(pose->orientation);
    rotateAroundOrigin(img, yaw, origin);
    translateImage(img, pose->position);

    if (gui) {
      cv::resize(img, resized_img, cv::Size(300,300)); 
      cv::imshow("Transformed image", resized_img);
      ROS_INFO("[transformations/transformCvImageByPose]: Press ESC to continue");
      while(1){
        int k = cv::waitKey(100);
        if(k==27)    // Esc key to stop
            break;
        else if(k==-1)  // normally -1 returned,so don't print it
            continue;
      }
    }

   // ros::Time end_time = ros::Time::now();
   // ros::Duration dur = end_time - begin_time;
   // ROS_INFO("[transformations/transformCvImageByPose]:  it took %f seconds.", dur.toSec());
  }
  //}
  
  /* copyAndTransformCvImageByPose() //{ */
  cv_bridge::CvImagePtr copyAndTransformCvImageByPose(const cv_bridge::CvImagePtr img_map, const geometry_msgs::PoseConstPtr pose, geometry_msgs::Pose& origin){
    cv_bridge::CvImagePtr new_img_map = boost::make_shared<cv_bridge::CvImage>(*img_map);
    transformCvImageByPose(new_img_map, pose, origin);
    return new_img_map;
  }
  //}

/* OpenCV //{ */

  void rotateAroundOrigin(cv::Mat src, double angle, const geometry_msgs::Pose& origin) {
    // The compounding operation of transforming point by pose is rotation the point around the origin of the occupancy grid map:
    cv::Point2f center(0., 0.);
    double degrees = -angle * 180/M_PI;
    //ROS_INFO("rotation in degrees=%f", degrees);
    cv::Mat rot = cv::getRotationMatrix2D(center, degrees, 1.0);
    cv::warpAffine(src, src, rot, src.size());
  }

  void translateImage(cv::Mat src, const geometry_msgs::Point& translation) {
    // Create translation matrix that looks like this
    // M = | 1 0 x |
    //     | 0 1 y |
    //ROS_INFO_STREAM("Translating image of size ("<< src.cols << ", " << src.rows << ") by x=" << translation.x << " and y=" << translation.y << " pixels.");   
    cv::Mat trans = (cv::Mat_<double>(2,3) << 1,0, translation.x, 0, 1, translation.y);
    //cv::Mat trans = (cv::Mat_<double>(2,3) << 1,0, 0, 0, 1, 0);
    cv::warpAffine(src, src, trans, src.size());
  }

//}
  
  /* YAW OPERATIONS //{ */

  double getYaw(const geometry_msgs::Quaternion& q) {
    tf2::Quaternion q_tf2;
    tf2::fromMsg(q, q_tf2);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_tf2).getRPY(roll, pitch, yaw);
    return yaw;
  }
 
  void setYaw(const double& yaw_new, geometry_msgs::Quaternion& q) {
    tf2::Quaternion q_tf2;
    tf2::fromMsg(q, q_tf2);
    double roll, pitch, yaw_old;
    tf2::Matrix3x3(q_tf2).getRPY(roll, pitch, yaw_old);
    q_tf2.setRPY(roll, pitch, yaw_new);
    q_tf2.normalize();
    q = tf2::toMsg(q_tf2);
  }
  
  //}

/* QUATERNION OPERATIOS //{ */

void setRPY(const double& roll, const double& pitch, const double& yaw, geometry_msgs::Quaternion& q) {
  tf2::Quaternion q_tf;
  q_tf.setRPY(roll, pitch, yaw);
  q = tf2::toMsg(q_tf);
}

void getRPY(geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw) {
  tf2::Quaternion q_tf;
  tf2::fromMsg(q, q_tf);
  tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);
}

//}

/* remapVarToUnitInterval() //{ */

float remapVarToUnitInterval(float x, float min, float max) {
  float len = max - min;
  return (x-min) / len;
}

//}

} // namespace merger2d

