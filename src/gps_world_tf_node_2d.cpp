#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <novatel_gps_msgs/NovatelXYZ.h>
#include <novatel_gps_msgs/NovatelUtmPosition.h>
#include <novatel_gps_msgs/Inspva.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/GPSFix.h>
#include <mmc_msgs/localization2D_msg.h>

#include <math.h>
#include <UTM.h>

#include <proj.h>

class gps_world_tf_node
{
private:
  /* data */
public:

  ros::NodeHandle node;
  ros::Subscriber sub1;
  ros::Subscriber sub2;
  ros::Subscriber sub3;
  ros::Publisher pub1;
  ros::Publisher pub2;
  ros::Publisher pub3;
  tf::Vector3 xyz_init;

  sensor_msgs::NavSatFix fix_msg;
  double r, theta_init, s;
  double N_init,E_init,H_init;
  double yaw_stored;

  tf2::Quaternion quat_init;
  geometry_msgs::TransformStamped static_init_transformStamped;
  geometry_msgs::TransformStamped gps_transformStamped;
  int Z_init;
  bool initialized = false;
  bool initialized_quat = false;
  int count;
  tf::Quaternion q;
  double height_current;
  gps_world_tf_node(/* args */);
  ~gps_world_tf_node();
  void fixCallback(const gps_common::GPSFixConstPtr& msg);
  void inspvaCallback(const novatel_gps_msgs::InspvaConstPtr& msg);
  void bestutmCallback(const novatel_gps_msgs::NovatelUtmPositionConstPtr& msg);
  void latlontoEPSG5179XY(double lat, double lon, double &x, double &y);
  void latlonalttoEPSG5179XYZ(double lat, double lon,double alt, double &x, double &y, double &z);

  PJ_CONTEXT *C_proj;
  PJ *P_proj;

  ros::Time past_time;
  tf::Transform transform_init;
};

gps_world_tf_node::gps_world_tf_node(/* args */)
{
  sub1 = node.subscribe("/sensors/gps/gps", 10, &gps_world_tf_node::fixCallback, this);
  sub2 = node.subscribe("/sensors/gps/inspva", 10, &gps_world_tf_node::inspvaCallback, this);


  pub1 = node.advertise<sensor_msgs::NavSatFix>("/sensors/gps/xyzinit",10000,true);
  // pub2 = node.advertise<mmc_msgs::localization2D_msg>("/localization/pose_2d",  1);
  pub3 = node.advertise<mmc_msgs::localization2D_msg>("/localization/pose_2d_gps",  1);

  H_init = 1.5;
  count = 0;
  C_proj = proj_context_create();
  P_proj = proj_create_crs_to_crs (C_proj, "EPSG:4326", "epsg:5179", NULL);

}

gps_world_tf_node::~gps_world_tf_node()
{
}

void gps_world_tf_node::latlontoEPSG5179XY(double lat, double lon, double &x, double &y)
{

  PJ *norm_proj;
  PJ_COORD a, b;
  
  /* This will ensure that the order of coordinates for the input CRS */
  /* will be longitude, latitude, whereas EPSG:4326 mandates latitude, */
  /* longitude */
  norm_proj = proj_normalize_for_visualization(C_proj, P_proj);

  /* a coordinate union representing Copenhagen: 55d N, 12d E    */
  /* Given that we have used proj_normalize_for_visualization(), the order of
  /* coordinates is longitude, latitude, and values are expressed in degrees. */
  a = proj_coord(lon, lat, 0, 0);

  /* transform to UTM zone 32, then back to geographical */
  b = proj_trans(norm_proj, PJ_FWD, a);

  /* Clean up */
  // proj_destroy(P_proj);
  // proj_context_destroy(C_proj);
  x = b.enu.e;
  y = b.enu.n;
}

void gps_world_tf_node::latlonalttoEPSG5179XYZ(double lat, double lon, double alt, double &x, double &y, double &z)
{
  PJ *norm_proj;
  PJ_COORD a, b;
  
  /* This will ensure that the order of coordinates for the input CRS */
  /* will be longitude, latitude, whereas EPSG:4326 mandates latitude, */
  /* longitude */
  norm_proj = proj_normalize_for_visualization(C_proj, P_proj);

  /* a coordinate union representing Copenhagen: 55d N, 12d E    */
  /* Given that we have used proj_normalize_for_visualization(), the order of
  /* coordinates is longitude, latitude, and values are expressed in degrees. */
  a = proj_coord(lon, lat, alt, 0);

  /* transform to UTM zone 32, then back to geographical */
  b = proj_trans(norm_proj, PJ_FWD, a);

  /* Clean up */
  proj_destroy(P_proj);
  proj_context_destroy(C_proj);
  x = b.enu.e;
  y = b.enu.n;
  z = b.enu.u;
}

void gps_world_tf_node::fixCallback(const gps_common::GPSFixConstPtr& msg){
  if (initialized_quat && initialized){
    
    double N,E;
    // auto before = ros::Time::now();
    latlontoEPSG5179XY(msg->latitude, msg->longitude, E, N);
    // std::cout << "proj time is: " << ros::Time::now() - before <<std::endl;
    double E_test, N_test, U_test;
    // latlonalttoEPSG5179XYZ(msg->latitude, msg->longitude, msg->altitude ,E_test, N_test,U_test);
    // std::cout << "E: "<< E_test <<", N: "<< N_test <<", U: "<<U_test <<std::endl;
    mmc_msgs::localization2D_msg msg_pose;
    msg_pose.time = msg->header.stamp;
    msg_pose.EPSG = 5179;
    msg_pose.east = E;
    msg_pose.north = N;
    msg_pose.gps_initialized = true;
    msg_pose.v = msg->speed;
    msg_pose.yaw = yaw_stored;
    msg_pose.sig_east = msg->position_covariance[0];
    msg_pose.sig_north = msg->position_covariance[4];
    // msg_pose.sig_east = msg->position_covariance[0,0];
    pub3.publish(msg_pose);

    static tf2_ros::TransformBroadcaster br_;
    gps_transformStamped.header.stamp = msg->header.stamp;
    gps_transformStamped.header.frame_id = "epsg5179";
    gps_transformStamped.child_frame_id = "gps";
    gps_transformStamped.transform.translation.x = E;
    gps_transformStamped.transform.translation.y = N;
    gps_transformStamped.transform.translation.z = 1.5;
    if (count%2 == 0)
    {
      br_.sendTransform(gps_transformStamped);
      count++;
    }else{
      count = 0;
    }
      

  } 
  else
  {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    latlontoEPSG5179XY(msg->latitude, msg->longitude, E_init, N_init);
    if (initialized_quat)
      initialized = true;
    static_init_transformStamped.header.stamp = ros::Time::now();
    static_init_transformStamped.header.frame_id = "epsg5179";
    static_init_transformStamped.child_frame_id = "exec_init";
    static_init_transformStamped.transform.translation.x = E_init;
    static_init_transformStamped.transform.translation.y = N_init;
    static_init_transformStamped.transform.translation.z = H_init;

    static_broadcaster.sendTransform(static_init_transformStamped);

    transform_init.setRotation(q);   
    transform_init.setOrigin(tf::Vector3(E_init, N_init, H_init));
  }
}

void gps_world_tf_node::inspvaCallback(const novatel_gps_msgs::InspvaConstPtr& msg){
  if (!initialized_quat){
    
    fix_msg.latitude = msg->latitude;
    fix_msg.longitude = msg->longitude;
    fix_msg.header.frame_id = "exec_init";
    
    pub1.publish(fix_msg);
    // LatLonToUTMXY(msg->latitude, msg->longitude, 52, E_init, N_init);

    

    
    quat_init.setRPY(msg->pitch/180*3.141592, msg->roll/180*3.141592, 1.57-msg->azimuth/180*3.141592);
    static_init_transformStamped.transform.rotation.x = quat_init.x();
    static_init_transformStamped.transform.rotation.y = quat_init.y();
    static_init_transformStamped.transform.rotation.z = quat_init.z();
    static_init_transformStamped.transform.rotation.w = quat_init.w();
    initialized_quat = true;
    yaw_stored = 1.57-msg->azimuth/180*3.141592;
    q.setRPY(msg->pitch/180*3.141592, msg->roll/180*3.141592,1.57-msg->azimuth/180*3.141592);
    return;
    
  }
  static tf::TransformBroadcaster br_ins;
  tf::Transform transform;
  q.setRPY(msg->pitch/180*3.141592, msg->roll/180*3.141592,1.57-msg->azimuth/180*3.141592);
  gps_transformStamped.transform.rotation.x = q.x();
  gps_transformStamped.transform.rotation.y = q.y();
  gps_transformStamped.transform.rotation.z = q.z();
  gps_transformStamped.transform.rotation.w = q.w();
  transform.setRotation(q);
  double N,E;
  int Z;
  latlontoEPSG5179XY(msg->latitude, msg->longitude, E, N);
  transform.setOrigin(tf::Vector3(E, N, 0));
  if (past_time != ros::Time::now())
    br_ins.sendTransform(tf::StampedTransform(transform_init.inverse()*transform, ros::Time::now(), "exec_init", "imu"));
  past_time = ros::Time::now();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "gps_world_tf");
  gps_world_tf_node gps_world_tf = gps_world_tf_node();
  ros::spin();
  return 0;
};