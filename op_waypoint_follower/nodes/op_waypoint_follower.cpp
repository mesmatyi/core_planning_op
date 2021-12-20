#include "op_waypoint_follower_core.h"

void callback(op_waypoint_follower::pid_tuneConfig &config, uint32_t level) {
	k_P = config.k_P;
	k_I = config.k_I;
	k_D = config.k_D;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "op_waypoint_follower");

	dynamic_reconfigure::Server<op_waypoint_follower::pid_tuneConfig> server;
  	dynamic_reconfigure::Server<op_waypoint_follower::pid_tuneConfig>::CallbackType f;

  	f = boost::bind(&callback, _1, _2);
  	server.setCallback(f);

	op_waypoint_follower::WaypointFollower wp_follow;
	wp_follow.RunMainLoop();
	return 0;
}
