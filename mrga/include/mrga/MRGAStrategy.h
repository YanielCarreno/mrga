#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>
#include "diagnostic_msgs/KeyValue.h"
#include "geometry_msgs/PoseStamped.h"
#include "mrga_msgs/allocation.h"
#include "mrga_msgs/CapabilityAnalyser.h"
#include "mrga_msgs/RegionsDelimiter.h"

#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <algorithm>
#include <iostream>
#include <stdlib.h>

namespace MRGA_approach {

	class MRGAStrategy
	{

	private:

		// MRGA Algorithm Services
		ros::ServiceClient service_capabilities_analyser;
		ros::ServiceClient service_regions_delimiter;
		ros::ServiceClient service_facts_declaration;

		// visualisation
		std::string fixed_frame;
		void publishWaypointMarkerArray(ros::NodeHandle nh);

	public:

		/* constructor */
		MRGAStrategy(ros::NodeHandle &nh, std::string frame);

		/* service to (re)generate waypoints */
		bool setupCoordinates(std::string goal_filename, std::string filename, std::string robot_filename);
	};
}
