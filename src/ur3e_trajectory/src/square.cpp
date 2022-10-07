#include "../include/square.hpp"

int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "square");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;
    
    // Create PlanningOptions
    MoveitPlanning::PlanningOptions planning_options =
    MoveitPlanning::PlanningOptions();
    planning_options.num_attempts = 10;
    planning_options.allow_replanning = true;
    planning_options.set_planning_time = 30.0;
    planning_options.goal_position_tolerance = 0.01;
    planning_options.goal_orientation_tolerance = 0.01;
    planning_options.goal_joint_tolerance = 0.01;
    planning_options.velocity_scaling_factor = 0.1;
    planning_options.acceleration_scaling_factor = 0.1;

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    //Write your code for following the square trajectory here.
     
    
}