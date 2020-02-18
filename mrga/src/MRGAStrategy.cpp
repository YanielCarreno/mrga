#include "mrga/MRGAStrategy.h"
//==============================================================================
/* Implementation of the Multi-Role Goal Assignment Strategy */
//==============================================================================
namespace MRGA_approach {
	//============================================================================
	/* constructor */
	//============================================================================
	MRGAStrategy::MRGAStrategy(ros::NodeHandle &nh, std::string frame)
	 :fixed_frame(frame) {
		//==========================================================================
 		/* service call to implement the goal allocation */
 		//==========================================================================
		service_capabilities_analyser = nh.serviceClient<mrga_msgs::CapabilityAnalyser>("/capabilities_analyser");
		service_regions_delimiter = nh.serviceClient<mrga_msgs::RegionsDelimiter>("/regions_delimiter");
	}


	bool MRGAStrategy::setupCoordinates(std::string goal_filename, std::string filename, std::string robot_filename) {
		//==========================================================================
		/* Capabilities Analyser */
		//==========================================================================
    //==========================================================================
		/* Variables declaration */
		//==========================================================================
		ros::NodeHandle nh("~");
		std::ifstream infile(filename.c_str());
		std::ifstream infile_robot(robot_filename.c_str());
		std::ifstream infile_goal(goal_filename.c_str());
		std::string goal_line;
		int goal_curr, goal_next, goal_step;
		//==========================================================================
		/* Goal Allocation: Capabilities Analyser */
		//==========================================================================
		mrga_msgs::CapabilityAnalyser srv_capability_analyser;
		srv_capability_analyser.request.robot_file = robot_filename;
		srv_capability_analyser.request.goals_file = goal_filename;
		//==========================================================================
		/* Call Service */
		//==========================================================================
		service_capabilities_analyser.waitForExistence();
		if (service_capabilities_analyser.call(srv_capability_analyser)){
			ROS_INFO("MRGA: Capability Analyser service is connected");
			bool capability_distribution = srv_capability_analyser.response.result;
			int robots_number = srv_capability_analyser.response.robot_no;
			if(capability_distribution == true){
				//======================================================================
				/* Goal Allocation: Regions Delimiter */
				//======================================================================
				mrga_msgs::RegionsDelimiter srv_regions_delimiter;
				srv_regions_delimiter.request.robots_number = robots_number;
				srv_regions_delimiter.request.waypoints_file = filename;
				srv_regions_delimiter.request.robot_file = robot_filename;
				srv_regions_delimiter.request.goals_file = goal_filename;
				srv_regions_delimiter.request.allocation_approach = "MRGA";
				//======================================================================
				/* Call Service */
				//======================================================================
				service_regions_delimiter.waitForExistence();
				if (service_regions_delimiter.call(srv_regions_delimiter)){
					ROS_INFO("MRGA: Regions Delimiter service is connected");
					bool goals_allocation = srv_regions_delimiter.response.result;
					int constriants_no = srv_regions_delimiter.response.constraints_no;
					if(goals_allocation == true){
						ROS_INFO("MRGA: GOAL WERE DISTRIBUTED!");
					}
					else{
						ROS_INFO("MRGA: Goal Allocation was not affective implemented");
					}
				}
				else{
					ROS_ERROR("MRGA: Regions Delimiter was not properly called");
				}
			}
			else{
				ROS_INFO("MRGA: Capability Analyser didn't find the index distribution");
			}
		}
		else{
			ROS_ERROR("MRGA: Capability Analyser was not properly called");
		}
	}
}

	//============================================================================
	/* Main method */
	//============================================================================
	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_simple_map_server");
		ros::NodeHandle nh("~");
		//==========================================================================
		// Definition of system parameters
		//==========================================================================
		std::string filename("mrga_waypoints.txt");
		std::string goal_filename("mrga_goals.txt");
		std::string robot_filename("mrga_robots.txt");
		std::string fixed_frame("world");
		nh.param("waypoint_file", filename, filename);
		nh.param("goal_file", goal_filename, goal_filename);
		nh.param("robot_file", robot_filename, robot_filename);
		nh.param("fixed_frame", fixed_frame, fixed_frame);
		//==========================================================================
		// initialisation
		//==========================================================================
		MRGA_approach::MRGAStrategy sms(nh, fixed_frame);
		sms.setupCoordinates(goal_filename, filename, robot_filename);
		ROS_INFO("KCL: (MRGAStrategy) Ready to receive.");
		ros::spin();
		return 0;
	}
