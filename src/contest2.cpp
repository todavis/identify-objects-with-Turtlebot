#include "ros/ros.h"
#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <nav_msgs/GetPlan.h> 
#include <std_srvs/Empty.h>
#include <fstream>

// To Test:
// roslaunch mie443_contest2 turtlebot_world.launch world:=practice
// roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/mnt/catkin_ws/src/mie443_contest2/maps/map_practice.yaml
// roslaunch turtlebot_rviz_launchers view_navigation.launch
// rosrun mie443_contest2 contest2

float GetDistance(RobotPose start_pt, RobotPose end_pt){
    // Change in dist
    float dx = start_pt.x - end_pt.x; 
    float dy = start_pt.y - end_pt.y;
    float dist = std::sqrt(dx * dx + dy * dy);
    return dist;
}

std::vector<int> GetGreedySearch(Boxes &box, RobotPose &pose){
    std::vector<int> node;
    std::vector<int> path;
    
    int j = 0; 
    for (auto& landmark : box.coords){
        node.push_back(j);
        j ++;
    }

    RobotPose curPose(pose.x, pose.y, pose.phi, 0);

    while (!node.empty()){
        float min_dist = 100;
        int best_i = 0;
        int best_j = 0;
        int j = 0;

        for (int i : node){
            RobotPose endPt(box.coords[i][0], box.coords[i][1], box.coords[i][2], 0);

            float dis = GetDistance(curPose, endPt);
            if (dis < min_dist){
                min_dist = dis;
                best_i = i;
                best_j = j;
            }
            j++;
        }
        curPose.x   = box.coords[best_i][0];
        curPose.y   = box.coords[best_i][1];
        curPose.phi = box.coords[best_i][2];

        path.push_back(best_i);
        node.erase(node.begin() + best_j);
    }
    return path;
}
int callPlanningService(ros::ServiceClient &serviceClient, float start_x, float start_y, float start_phi, float goal_x, float goal_y, float goal_phi)
{
    nav_msgs::GetPlan srv;
    srv.request.start.header.frame_id ="map";
    srv.request.start.pose.position.x = start_x;
    srv.request.start.pose.position.y = start_y;
    srv.request.start.pose.orientation.w = start_phi;
    srv.request.goal.header.frame_id = "map";
    srv.request.goal.pose.position.x = goal_x;
    srv.request.goal.pose.position.y = goal_y;
    srv.request.goal.pose.orientation.w = goal_phi;
    srv.request.tolerance = 0.0;
    
    if (serviceClient.call(srv)) {
        if (!srv.response.plan.poses.empty()) {
            return 1;
        } else {
            ROS_INFO("Plan Failed!");
            return 0;
        }
    }
    else {
        ROS_INFO("Service Call Failed!");
        return 0;
    }
}


int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    RobotPose robotPose(0,0,0,0);

    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    ros::ServiceClient make_plan_client = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan", true);
    ros::ServiceClient clear_cost_client = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    
    if (!make_plan_client) {
        ROS_FATAL("Could not initialize get plan service from %s", make_plan_client.getService().c_str());
        return -1;
    }
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    /* for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    } */

    ImagePipeline imagePipeline(n);
    Navigation nav;
    // Execute strategy.
    std::string state = "localizing";
    std::ofstream id_file;
    std::vector<int> seen_ids;

    int done_travel = 0;
    int done_imaging = 0;
    int found_path = 0;
    int restart = 0;
    float dist = 0.3;
    int box = 0;
    int rth = 0;

    float ang = 0.0;
    float goal_x = 0.0;
    float goal_y = 0.0;
    float goal_phi = 0.0;

    float start_x = 0.0;
    float start_y = 0.0;
    float start_phi = 0.0;

    std::vector<int> greedy_path;

    id_file.open("output.txt", std::ios::out | std::ios::app);
    id_file << "Start run\n";
    id_file.close();

    while(ros::ok()) {
        ros::spinOnce();
        if(state == "localizing"){
            // Localization state
            std::cout << "Robot Pose Uncertainty: " << robotPose.cov << std::endl;
            nav.moveToGoal(robotPose.x, robotPose.y, std::fmod(robotPose.phi + M_PI/3, 2*M_PI));

        } else if (state == "planning"){
            std::cout << "Start Pose: x: " << robotPose.x<< " y: " << robotPose.y << " phi: " << robotPose.phi << std::endl;
            
            ang = boxes.coords[box][2];
            goal_x = boxes.coords[box][0] + std::cos(ang)*dist;
            goal_y = boxes.coords[box][1] + std::sin(ang)*dist;
            goal_phi = std::fmod(ang + M_PI, 2*M_PI);

            if (callPlanningService(make_plan_client, robotPose.x, robotPose.y, robotPose.phi, goal_x, goal_y, goal_phi)){
                found_path = 1;
            } else {
                for(float offset = 0.1; offset <= 1.5; offset += 0.1){
                    for (int sign = 1; sign > -2; sign -= 2){
                        ang = boxes.coords[box][2] + sign*offset;
                        goal_x = boxes.coords[box][0] + std::cos(ang)*dist;
                        goal_y = boxes.coords[box][1] + std::sin(ang)*dist;
                        goal_phi = std::fmod(ang + M_PI, 2*M_PI);
                        if (callPlanningService(make_plan_client, robotPose.x, robotPose.y, robotPose.phi, goal_x, goal_y, goal_phi)){
                            found_path = 1;
                            break;
                        }    
                    }
                    if (found_path){
                        break;
                    }
                }
            }

        } else if (state == "traveling"){
            if (nav.moveToGoal(goal_x, goal_y, goal_phi)){
                done_travel = 1;
            } else {
                restart = 1;
            }
        } else if (state == "imaging"){
            // Imaging state
            int temp_id;
            temp_id = imagePipeline.getTemplateID(boxes);
            
            id_file.open("output.txt", std::ios::out | std::ios::app);
            
            id_file << "Tag: " << temp_id;
            id_file << " x: " << boxes.coords[box][0];
            id_file << " y: " << boxes.coords[box][1];
            id_file << " phi: " << boxes.coords[box][2];
            
            if (std::find(seen_ids.begin(), seen_ids.end(), temp_id) != seen_ids.end()) {
                id_file << " Duplicate: true\n";
            } else { 
                id_file << " Duplicate: false\n";
                seen_ids.push_back(temp_id);
            }
            id_file.close();
            done_imaging = 1;

        }  else if (state == "end"){
            id_file.open("output.txt", std::ios::out | std::ios::app);
            id_file << "End of the run\n";
            id_file.close();
            return 0;
        }
        
        // State Transition
        if (state == "localizing" && robotPose.cov < 0.02){
            std::cout << "Localization complete"<< std::endl;
            state = "planning";
            if (restart || dist > 3){
                restart = 0;
                dist = 0.3;
            } else {
                start_x = robotPose.x;
                start_y = robotPose.y;
                start_phi = robotPose.phi;
                RobotPose startPose(robotPose.x, robotPose.y, robotPose.phi, 0);
                greedy_path = GetGreedySearch(boxes, startPose);
                box = greedy_path[0];
                greedy_path.erase(greedy_path.begin());
            }
            
            
        } else if (state == "planning" && found_path){
            std::cout << "Found Path: x: " << goal_x<< " y: " << goal_y << " phi: " << goal_phi << std::endl;
            state = "traveling";
            found_path = 0;

        } else if (state == "planning" && !found_path){
            std::cout << "Planning Failed" << std::endl;
            state = "planning";
            found_path = 0;
            ang = 0;
            goal_x = 0;
            goal_y = 0;
            goal_phi = 0;
            dist += 0.1;

        } else if (state == "planning" && dist > 3){
            std::cout << "Just give up, you're not getting to this location" << std::endl;
            state = "localizing";
            done_travel = 0;
            restart = 0;
            ang = 0;
            goal_x = 0;
            goal_y = 0;
            goal_phi = 0;

        }else if (state == "traveling" && rth && done_travel){
            std::cout << "Operation completed" << std::endl;
            state = "end";
            
        } else if (state == "traveling" && done_travel){
           std::cout << "Travel succeeded" << std::endl;
            state = "imaging";
            
        } else if (state == "traveling" && restart){
            std::cout << "Travel failed" << std::endl;
            state = "localizing";
            done_travel = 0;
            ang = 0;
            goal_x = 0;
            goal_y = 0;
            goal_phi = 0;
            dist = 0.3;
        } else if (state == "imaging" && greedy_path.empty() && done_imaging){
            std::cout << "return to home" << std::endl;
            goal_x = start_x;
            goal_y = start_y;
            goal_phi = start_phi;
            rth = 1;
            done_travel = 0;
            state = "traveling";
            
        } else if (state == "imaging" && done_imaging){
            std::cout << "Imaging finished" << std::endl;
            state = "planning";
            box = greedy_path[0];
            greedy_path.erase(greedy_path.begin());
            done_travel = 0;
            done_imaging = 0;
            restart = 0;
            ang = 0;
            goal_x = 0;
            goal_y = 0;
            goal_phi = 0;
            dist = 0.3;
        }          
        ros::Duration(0.01).sleep();
    }
    return 0;
}
