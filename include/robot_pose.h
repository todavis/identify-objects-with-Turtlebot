#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>

class RobotPose {
	public:
		float x;
		float y;
		float phi;
		float cov;
	public:
		RobotPose(float x, float y, float phi, float cov);
		void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
};
