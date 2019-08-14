/*Created by Daila De Santis and Mahdieh Nejati
Script for simulating different multi-stage tajectories given start and goal poses. 
*/

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/String.h>
#include <vector>

//#include <tf/Pose.h>

#include <kinova_driver/kinova_ros_types.h>

namespace JacoBoMI{
class trajectory_simulator
{
    ros::NodeHandle nh_;

    tf::Pose Home_;
    tf::Pose Goal_;
    geometry_msgs::Pose target_pose_;
 
    //moveit::planning_interface::MoveGroup group;
    moveit_msgs::DisplayTrajectory display_trajectory;
    //moveit::planning_interface::MoveGroup::Plan my_plan;

    std::vector<double> group_joint_values_;
    std::vector<double> home_joint_values_;
    robot_state::RobotState *robot_state_;
    robot_state::RobotState *start_state_;

    int N_Joints = 7;

    void initialize_publishers()
    {
        display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        // display_publisher = nh_.advertise<std_msgs::String>("/test_topic", 1, true);

    }

    public:

    ros::Publisher display_publisher;
    moveit::planning_interface::MoveGroup group;
    moveit::planning_interface::MoveGroup::Plan my_plan;

    trajectory_simulator(): group("arm")
    {
        ROS_INFO("Initializing publisher!!");
        initialize_publishers();
        home_joint_values_.reserve(N_Joints);
        robot_state_ = new robot_state::RobotState(group.getRobotModel());
        start_state_ = new robot_state::RobotState(group.getRobotModel());
    };

    ~trajectory_simulator()
    {
        ROS_INFO("Bye!!");        // ^^^^^^^^^^^^^^^^^^^^^^^
        // We can plan a motion for this group to a desired pose for the 
        // end-effector.
    };

    /*Read Configuration File(s)*/
    void configure()
    {
        ROS_INFO("Working!!");
        double defaultHome[] = {4.71,2.84,0.00,0.75,4.62,4.48,4.88};
        std::vector<double>::iterator it = home_joint_values_.begin();
        for (int i = 0; i<7; i++, it++)
        {
            (*it) = defaultHome[i];
        }
        //home_joint_values_.assign(defaultHome,defaultHome+7);
    };

    
    void setHome(tf::Vector3 _origin, tf::Quaternion _Q)
    {
        Home_.setOrigin(_origin);
        Home_.setRotation(_Q);
        ROS_INFO("Home set!!");
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(5.0);
    };

    void HomeRobot()
    {
        // get currento state

        //ROS_INFO("Current start state:");      
        //group.setStartStateToCurrentState();
        //group.setStartState(*start_state_);
        //start_state_->printStatePositions();

        //// move to start state = Home (visualization)
        //tf::poseTFToMsg(Home_, target_pose_);
        //ROS_INFO("target pose = Home");
        //group.setPoseTarget(target_pose_);
        //bool success = (group.plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
        //ROS_INFO("Visualizing plan 2 Home (pose goal) %s",success?"":"FAILED");   
        /* Sleep to give Rviz time to visualize the plan. */
        //sleep(5.0);
        //if (success)
        //    group.move();
        //sleep(5.0);

        // Now set start pose to home
        start_state_->setJointGroupPositions(group.getName(),home_joint_values_);
        group.setStartState(*start_state_);
        ROS_WARN("start_state after set: ");
        start_state_->printStatePositions();
    };

    void setGoal(tf::Vector3 _origin, tf::Quaternion _Q)
    {
        Goal_.setOrigin(_origin);
        Goal_.setRotation(_Q);
        ROS_INFO("Goal set!!");
    };

    void setGoalAsHome()
    {
        Goal_.setOrigin(Home_.getOrigin());
        Goal_.setRotation(Home_.getRotation());
        ROS_INFO("Goal set = Home!!");
    };

    /*Free trajectory planning*/
    void PlanFreeMotion()
    {
        ROS_INFO("Planning Motion!");
        // Planning to a Pose goal
        // ^^^^^^^^^^^^^^^^^^^^^^^
        // We can plan a motion for this group to a desired pose for the 
        // end-effector.
        tf::poseTFToMsg(Goal_, target_pose_);
        ROS_INFO("target pose");
        group.setPoseTarget(target_pose_);
        geometry_msgs::PoseStamped tp = group.getPoseTarget();
        ROS_INFO("target pose set to: x=%f \n y=%f\n z=%f\n xr=%f\n yr=%f\n zr=%f\n w=%f\n ", 
        tp.pose.position.x, tp.pose.position.y, tp.pose.position.z, 
        tp.pose.orientation.x, tp.pose.orientation.y, tp.pose.orientation.z, tp.pose.orientation.w);

        // Now, we call the planner to compute the plan
        // and visualize it.
        // Note that we are just planning, not asking move_group 
        // to actually move the robot.

        //bool success = errorcode.val == 1;
        bool success = (group.plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);

        ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(5.0);

        // TODO
        //if success update start state to funal current state.
    };

    void VisualizePlan()
    {
        ROS_INFO("Visualizing plan");    
        display_trajectory.trajectory_start = my_plan.start_state_;
        display_trajectory.trajectory.push_back(my_plan.trajectory_);
        display_publisher.publish(display_trajectory);
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(5.0);

    };

    void MoveRobot2Goal()
    {
        // Moving to a pose goal
        // ^^^^^^^^^^^^^^^^^^^^^
        //
        // Moving to a pose goal is similar to the step above
        // except we now use the move() function. Note that
        // the pose goal we had set earlier is still active 
        // and so the robot will try to move to that goal. We will
        // not use that function in this tutorial since it is 
        // a blocking function and requires a controller to be active
        // and report success on execution of a trajectory.

        group.move();
        sleep(5.0);
    };
    /**/
    /**/
};

}




using namespace JacoBoMI;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_simulator");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /* This sleep is ONLY to allow Rviz to come up */
    //sleep(10.0);

    trajectory_simulator tgsim;
    tgsim.configure();

    tf::Vector3 hposit(1.63771402836,1.11316478252, 0.134094119072);
    tf::Quaternion horient (kinova::EulerXYZ2Quaternion(0.212322831154, -0.257197618484, 0.509646713734));
    tgsim.setHome(hposit,horient);

    tgsim.HomeRobot(); // set start state
    


    tf::Vector3 gposit(0.54882, -0.30854,  0.65841);
    tf::Quaternion gorient(0.68463, -0.22436, 0.68808, 0.086576);
    tgsim.setGoal(gposit,gorient);
    tgsim.PlanFreeMotion(); 
    //tgsim.MoveRobot2Goal();

    tgsim.setGoalAsHome();
    tgsim.PlanFreeMotion(); 
    //tgsim.MoveRobot2Goal();

    //tgsim.VisualizePlan();


    // std_msgs::String a_string;
    // a_string.data = "hi";

    // while (ros::ok())
    // {
    //     tgsim.display_publisher.publish(a_string); 
    //     ros::spinOnce();
    //     sleep(10.0);
    // }

    
    ROS_INFO("Bye!!");
    ros::shutdown();

  return 0;
}
