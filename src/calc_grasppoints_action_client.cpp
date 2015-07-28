/*
 * David Fischinger - Vienna University of Technology
 * March 2015
 *
 * ** Grasp Action Client - for Testing **
 *
 * This node is serving as a client for the calculation of grasp points
 * based on the action server for grasp calculation.
 * This nodes subscribes to the topic /haf_grasping/depth_registered/single_cloud/points_in_lcs (lcs: local coordinate system)
 * and receives (single) point clouds (in a local coordinate system where z is perpendicular to the floor).
 * The node sends the point cloud as an action server goal and receives the grasp result.
 *
 */



//ROS includes
#include <ros/ros.h>
#include <ros/package.h>
#include "sensor_msgs/PointCloud2.h"
#include <geometry_msgs/Point.h>
//actionlib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//grasp action message
#include <haf_grasping/CalcGraspPointsServerAction.h>
// service messages
#include <haf_grasping/GraspSearchCenter.h>
#include <haf_grasping/GraspSearchRectangleSize.h>
#include <haf_grasping/GraspCalculationTimeMax.h>
// for reading pcd file
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_ros/transforms.h"
#include <pcl_ros/point_cloud.h>
//#include "pcl/io/io.h"
//#include <boost/thread/thread.hpp>



class CCalcGrasppointsClient
{
public:
	ros::Subscriber pc_sub;								//subscriber for pointcloud
	ros::Subscriber pcd_sub;							//subscriber for path for pcd-file (to read)
	ros::ServiceServer srv_set_grasp_center;			// service to set new grasp center (center of rectangles where grasps are searched for)
	ros::ServiceServer srv_set_grasp_search_area_size;	// service to set size of rectangle where grasps are searched
	ros::ServiceServer srv_set_grasp_calculation_time_max;	// service to set maximal grasp calculation time (sec) before result is returned
	geometry_msgs::Point graspsearchcenter;				// center for searching for grasps
	int grasp_search_size_x;		// the size (x direction) where grasps are really calculated (in each direction 7cm more are needed for feature calculation!
	int grasp_search_size_y;		// the size (y direction) where grasps are really calculated (in each direction 7cm more are needed for feature calculation!
	int max_grasp_search_size_x;	// x-limit for grasp search area size
	int max_grasp_search_size_y;	// y-limit for grasp search area size
	ros::Duration grasp_calculation_time_max;	//max time used for grasp calculation (sec) before result is returned

	void get_grasp_cb(const sensor_msgs::PointCloud2ConstPtr& pc_in);
	void open_pcd_and_trig_get_grasp_cb(std_msgs::String pcd_path);
	bool set_grasp_center(haf_grasping::GraspSearchCenter::Request &req, haf_grasping::GraspSearchCenter::Response &res);
	bool set_grasp_search_area_size(haf_grasping::GraspSearchRectangleSize::Request &req, haf_grasping::GraspSearchRectangleSize::Response &res);
	bool set_grasp_calculation_time_max(haf_grasping::GraspCalculationTimeMax::Request &req, haf_grasping::GraspCalculationTimeMax::Response &res);
	CCalcGrasppointsClient(ros::NodeHandle nh_)
	{
		//define center of grasp search rectangle
		this->graspsearchcenter.x = 0.0;
		this->graspsearchcenter.y = 0.0;
		this->graspsearchcenter.z = 0.0;
		//define size of grasp search rectangle
		this->grasp_search_size_x = 32;					//max. limit 32
		this->grasp_search_size_y = 44;					//max. limit 44
		this->max_grasp_search_size_x = 18;				//max. limit 32-14=18
		this->max_grasp_search_size_y = 30;				//max. limit 44-14=30
		// define maximal time before grasp result is returned
		this->grasp_calculation_time_max = ros::Duration(50);	// in sec
		//subscriber for the point cloud
		this->pc_sub = nh_.subscribe("/haf_grasping/depth_registered/single_cloud/points_in_lcs",1, &CCalcGrasppointsClient::get_grasp_cb, this);
		this->pcd_sub = nh_.subscribe("/haf_grasping/input_pcd_rcs_path",1, &CCalcGrasppointsClient::open_pcd_and_trig_get_grasp_cb, this);
		this->srv_set_grasp_center = nh_.advertiseService("/haf_grasping/set_grasp_center", &CCalcGrasppointsClient::set_grasp_center,this);
		this->srv_set_grasp_search_area_size = nh_.advertiseService("/haf_grasping/set_grasp_search_area_size", &CCalcGrasppointsClient::set_grasp_search_area_size,this);
		this->srv_set_grasp_calculation_time_max = nh_.advertiseService("/haf_grasping/set_grasp_calculation_time_max", &CCalcGrasppointsClient::set_grasp_calculation_time_max,this);
	}
};

// open pcd file for given path and start get_grasp_cb (that triggers grasp calculation)
void CCalcGrasppointsClient::open_pcd_and_trig_get_grasp_cb(std_msgs::String pcd_path)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_path.data.c_str(), *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read input file (pcd) \n");
    return;
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from " << pcd_path.data.c_str();

  //transform pcl to ros-msg
  sensor_msgs::PointCloud2 pcd_as_ros_msg;// = new sensor_msgs::PointCloud2ConstPtr();
  pcl::toROSMsg(*cloud, pcd_as_ros_msg);
  pcd_as_ros_msg.header.frame_id = "base_link";
  pcd_as_ros_msg.header.stamp = ros::Time(0);
  const sensor_msgs::PointCloud2ConstPtr pcd_as_ros_msg_const_ptr = boost::make_shared<sensor_msgs::PointCloud2>(pcd_as_ros_msg);
  CCalcGrasppointsClient::get_grasp_cb(pcd_as_ros_msg_const_ptr);
}

// define goal (input) for grasp calculation, send it to grasp action server and receive result
void CCalcGrasppointsClient::get_grasp_cb(const sensor_msgs::PointCloud2ConstPtr& pc_in)
{
	ROS_INFO("\nFrom calc_grasppoints_action_client: point cloud received");

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<haf_grasping::CalcGraspPointsServerAction> ac("calc_grasppoints_svm_action_server", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	haf_grasping::CalcGraspPointsServerGoal goal;
	goal.graspinput.input_pc = *pc_in;

	goal.graspinput.grasp_area_center = this->graspsearchcenter;

	// set size of grasp search area
	goal.graspinput.grasp_area_length_x = this->grasp_search_size_x+14;
	goal.graspinput.grasp_area_length_y = this->grasp_search_size_y+14;

	// set max grasp calculation time
	goal.graspinput.max_calculation_time = this->grasp_calculation_time_max;

	//send goal
	ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(50.0));

	if (finished_before_timeout)
	{
	    actionlib::SimpleClientGoalState state = ac.getState();
	    boost::shared_ptr<const haf_grasping::CalcGraspPointsServerResult_<std::allocator<void> > > result = ac.getResult();
	    ROS_INFO("Result: %s", (*(result)).result.data.c_str());
	    ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	    ROS_INFO("Action did not finish before the time out.");
}


//set grasp search center
bool CCalcGrasppointsClient::set_grasp_center(haf_grasping::GraspSearchCenter::Request &req,
		haf_grasping::GraspSearchCenter::Response &res)
{
	//set grasp search center
	this->graspsearchcenter.x = req.graspsearchcenter.x;
	this->graspsearchcenter.y = req.graspsearchcenter.y;
	ROS_INFO("Set grasp search center to: x=%ld, y=%ld", (long int)req.graspsearchcenter.x, (long int)req.graspsearchcenter.y);
	res.result = true;
	ROS_INFO("sending back response: [%ld]", (long int)res.result);
	return res.result;
}

//set size of rectangle where grasps are searched
bool CCalcGrasppointsClient::set_grasp_search_area_size(haf_grasping::GraspSearchRectangleSize::Request &req,
		haf_grasping::GraspSearchRectangleSize::Response &res)
{
	//set grasp search rectangle size
	if (req.grasp_search_size_x >= 0 and req.grasp_search_size_x <= this->max_grasp_search_size_x){
		this->grasp_search_size_x = req.grasp_search_size_x;
		ROS_INFO("Set grasp rectangle size to: x=%ld", (long int)req.grasp_search_size_x);
	} else {
		ROS_INFO("Could not set grasp rectangle size for x. Allowed values: [0, %ld ]. Received value was: x=%ld", (long int)this->max_grasp_search_size_x,(long int)req.grasp_search_size_x);
		res.result = false;
		return res.result;
	}
	if (req.grasp_search_size_y >= 0 and req.grasp_search_size_y <= this->max_grasp_search_size_y){
		this->grasp_search_size_y = req.grasp_search_size_y;
		ROS_INFO("Set grasp rectangle size to: y=%ld", (long int)req.grasp_search_size_y);
	} else {
		ROS_INFO("Could not set grasp rectangle size for y. Allowed values: [0, %ld ]. Received value was: y=%ld", (long int)this->max_grasp_search_size_y,(long int)req.grasp_search_size_y);
		res.result = false;
		return res.result;
	}

	res.result = true;
	ROS_INFO("sending back response: [%ld]", (long int)res.result);
	return res.result;
}


//set maximal grasp calculation time before result has to be returned
bool CCalcGrasppointsClient::set_grasp_calculation_time_max(haf_grasping::GraspCalculationTimeMax::Request &req,
		haf_grasping::GraspCalculationTimeMax::Response &res)
{
	//set max grasp calculation time
	this->grasp_calculation_time_max = req.max_calculation_time;
	ROS_INFO("Set max calculation time (sec) to: x=%d", (int)req.max_calculation_time.toSec());
	res.result = true;
	ROS_INFO("sending back response: [%d]", (int)res.result);
	return res.result;
}




int main (int argc, char **argv)
{
  ROS_INFO("ROS NODE calc_grasppoints_client started");
  ros::init(argc, argv, "calc_grasppoint_client");
  ros::NodeHandle nh_;
  CCalcGrasppointsClient grasp_client(nh_);

  ros::spin();
  return 0;
}


