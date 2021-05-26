#include <robot_pose.h>
#include <tf/transform_datatypes.h>

RobotPose::RobotPose(float x, float y, float phi, float cov) {
	this->x = x;
	this->y = y;
	this->phi = phi;
	this->cov = cov;
}

void RobotPose::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
	phi = tf::getYaw(msg.pose.pose.orientation);
	x = msg.pose.pose.position.x;
	y = msg.pose.pose.position.y;
	cov = std::sqrt(msg.pose.covariance[0]*msg.pose.covariance[0] + msg.pose.covariance[7]*msg.pose.covariance[7] + msg.pose.covariance[35]*msg.pose.covariance[35]);

}
