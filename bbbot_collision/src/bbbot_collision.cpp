#include <bbbot_collision/CollisionCheckRequest.h>
#include "ros/ros.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "moveit/planning_scene/planning_scene.h"
#include "bbbot_collision/CollisionCheck.h"
#include "moveit/robot_model/robot_model.h"


class CollisionCheck {
protected:
    ros::NodeHandle nh_;
    planning_scene_monitor::PlanningSceneMonitorPtr psm_ptr_;
    ros::ServiceServer collison_check_ss_;

public:
    CollisionCheck() {
        // Wait till moveit has started
        // ros::service::waitForService("/move_group/plan_execution/set_parameters");
        std::string description;

        /*if(!nh_.getParam("robot_description", description)) {
            ROS_ERROR("Unable to find the robot_description");
            exit(1);
        }*/

        psm_ptr_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
        // Advertise the collision server
        collison_check_ss_ = nh_.advertiseService("collision_check", &CollisionCheck::collsionServiceCB, this);
        ROS_INFO("Collision Check service started");

    }

private:
    bool collsionServiceCB(bbbot_collision::CollisionCheck::Request& req,
        bbbot_collision::CollisionCheck::Response& resp) {


        // Get a readonly version of planning scene monitor
        planning_scene_monitor::LockedPlanningSceneRO scene(psm_ptr_);
        robot_model::RobotModelConstPtr robot_model = scene->getRobotModel();

        robot_trajectory::RobotTrajectoryPtr rtraj_ptr(new robot_trajectory::RobotTrajectory(robot_model, "arm"));
        rtraj_ptr->setRobotTrajectoryMsg(robot_model, req.trajectory);

        bool valid_path;
        valid_path = scene->isPathValid(*rtraj_ptr, "arm", true);
        ROS_INFO("Generated path is %s", valid_path ? "valid" : "invalid");
        resp.collision_found = !valid_path;
        return true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_check");
    ros::NodeHandle nh;

    CollisionCheck ccheck;

    ros::spin();
    return 0;
}
