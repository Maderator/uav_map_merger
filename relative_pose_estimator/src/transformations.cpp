

#include "transformations.h"
  
namespace relative_pose_estimator {

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

  /* compoundPose() method //{ */
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
  
  /* invertPose() method //{ */
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
} // namespace relative_pose_estimator

