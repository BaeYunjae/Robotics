// 필요한 헤더 파일 include
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

// rviz_visual_tools 줄여서 rvt로 정의
namespace rvt = rviz_visual_tools;

// joint 경로를 input으로 받아서 end-effector의 pose 경로를 알려주는 기능 함수 구현
void convert_JointTraj_to_PoseTraj(
    std::vector<geometry_msgs::Pose> &path,               // output으로 쓸 변수의 reference
    const robot_state::RobotStatePtr &rs,                 // FK을 계산하기 위한 
    const std::string &planning_group,                    // MoveIt 요소
    const trajectory_msgs::JointTrajectory &joint_traj)   // input으로 쓸 joint 경로
{
    // output clear, 필요한 MoveIt 요소인 JointModelGroup과 end-effector 이름 모음
    path.clear();
    const moveit::core::JointModelGroup *jmg = rs->getJointModelGroup(planning_group);
    const std::string &eef_name = jmg->getLinkModelNames().back();

    // Joint trajectory to Cartesian path (i == trajectory length)
    // joint 경로의 모든 점들에 대해 FK 풀기
    for (int i = 0; i < joint_traj.points.size(); i++)
    {
        // Joint trajectory to robot state
        // FK 풀어줄 JointModelGroup에 원하는 joint 값 설정, end-effector의 pose 구하기
        // JointModelGroup : 로봇의 기구학 정보 보유, 현재 로봇 상태와 별개로 joint 각도를 설정하여 연산 가능
        rs->setJointGroupPositions(jmg, joint_traj.points[i].positions);
        // Get the position of the end-effector from the RobotState
        Eigen::Affine3d eef_transformation = rs->getGlobalLinkTransform(eef_name);
        geometry_msgs::Pose eef_pose = Eigen::toMsg(eef_transformation);
        path.push_back(eef_pose);
    }
}

// main 함수에서는 먼저 ros node 초기화
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_node");
    ros::NodeHandle nh("/ik_node");
    ros::AsyncSpinner spinner(4);   // Use 4 threads
    spinner.start();
    static const std::string LOGNAME = "ik_node";

    // rosrun fk moveit fk_node _robot:=puma_560
    // getParam을 통해 robot 파라미터에 저장된 값을 planning_group으로 사용
    // 미설정 시 error, 종료
    int robot_type;
    std::string planning_group;
    if (nh.getParam("robot", planning_group))
    {
        ROS_INFO_STREAM_NAMED(LOGNAME, "Using planning group: " << planning_group);
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "No planning group specified");
        return 1;
    }
    // Waypoints: Sqaure trajectory (Joint-space)
    // 사각형 네 꼭짓점 중 첫번째 지점 선언
    geometry_msgs::Pose target_pose1;
    // 사각형 가로 세로 길이 dx, dy
    double dx, dy;

    // 입력된 robot 파라미터에 따라 target_pose1, dx, dy 설정, 의도되지 않은 값이 입력되면 error로 종료
    if (planning_group.compare("puma_560") == 0)
    {
        target_pose1.position.x = 0.3;
        target_pose1.position.y = -0.2;
        target_pose1.position.z = 0.6;
        tf2::Quaternion target_quat(tf2::Vector3(1.0, 0.0, 0.0), M_PI);
        target_pose1.orientation = tf2::toMsg(target_quat);
        dx = 0.4;
        dy = 0.4;
    }
    else if (planning_group.compare("rrr") == 0)
    {
        target_pose1.position.x = 0.25;
        target_pose1.position.y = -0.1;
        target_pose1.position.z = 0.02;
        target_pose1.orientation.w = 1.0;
        dx = 0.2;
        dy = 0.2;        
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Unsupported planning group: " << planning_group);
        return 1;
    }
    // setFromIK 함수가 입력을 Pose로만 받음, PoseStamped가 아니라 Pose의 vector 생성
    int num_waypoints = 4;

    std::vector<geometry_msgs::Pose> waypoints(num_waypoints, target_pose1);
    waypoints[1].position.x += dx;
    waypoints[2].position.x += dx;  
    waypoints[2].position.y += dy;
    waypoints[3].position.y += dy;

    // For waypoint visualization
    // Rviz에 waypoints를 닫힌 도형으로 그리기 위해 모든 요소를 straight_line으로 복사
    // 첫번째 요소를 마지막 요소로 한번 더 복사, straight_line 요소를 선으로 이으면 닫힌 도형
    std::vector<geometry_msgs::Pose> straight_line = waypoints;
    straight_line.push_back(waypoints[0]);

    // Visualization
    // Rviz에서 마커를 쉽게 그리기 위한 visual_tools 생성, 초기화
    std::string base_frame = "link1";
    moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // Setup for planning
    // MoveIt의 핵심 요소들을 생성, 초기화
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // MoveIt이 planning할 search space를 좌하단 xyz(-2, -2, 0), 우상단 xyz(2, 2, 3)의 박스로 설정
    move_group.setWorkspace(-2, -2, 0, 2, 2, 3);

    // Setup for IK    
    robot_state::RobotStatePtr kinematic_state(move_group.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup(planning_group);

    // Placeholder for joint values
    // 반복문으로 waypoint를 차례대로 순회하며 매 iteration마다 '현재의 joint값'과
    // 'end-effector가 다음 waypoint에 도달했을 때의 joint 값'을 spline으로 연결하여 Joint 경로를 구함
    // 역기구학(setFromIK) 결과를 저장할 target_joints 생성
    std::vector<double> target_joints(joint_model_group->getVariableCount());

    // Demo
    int i = 0;
    while (ros::ok())
    {
        // i번째 cartesian waypoint에 대해 역기구학 먼저 풀고, MoveIt의 joint space 경로탐색 결과와 비교
        ROS_INFO("< Planning to waypoint %d >", i);

        // IK
        const unsigned int attempts = 10;
        const double timeout = 0.1;
        bool found_ik = kinematic_state->setFromIK(joint_model_group, waypoints[i], attempts, timeout);
        // 역기구학 결과인 joint 각도를 터미널에 출력
        if (found_ik)
        {
            kinematic_state->copyJointGroupPositions(joint_model_group, target_joints);
            for (std::size_t i = 0; i < target_joints.size(); ++i)
            {
                ROS_INFO("\t(IK solution) Joint %d: %f", i, target_joints[i]);
            }
            // Jacobian도 출력
            Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
            Eigen::MatrixXd jacobian;
            kinematic_state->getJacobian(
                joint_model_group,
                kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                reference_point_position,
                jacobian);
            ROS_INFO_STREAM("Jacobian: \n"
                            << jacobian);
        }
        else
        {
            ROS_WARN("Did not find IK solution");
        }
    
        // Motion Planning
        // MoveIt이 joint 경로를 탐색하도록 joint 목표값을 입력
        move_group.setJointValueTarget(target_joints);
        // 탐색 결과 저장할 수 있도록 비어있는 plan 생성
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        // 탐색 수행, 수행결과(success)가 true이면, 경로는 my_plan에 저장되어 있음
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            // Path visualization
            // end-effector 궤적 구하고 각종 정보 시각화
            std::vector<geometry_msgs::Pose> eef_path;
            convert_JointTraj_to_PoseTraj(
                eef_path,
                kinematic_state,
                planning_group,
                my_plan.trajectory_.joint_trajectory);
            // 실제 end-effector 궤적 LIME_GREEN 색상으로 rviz 위에 그림, eef path를 그림
            visual_tools.publishPath(eef_path, rvt::LIME_GREEN, rvt::SMALL); 
            // end-effector가 거쳐갈 waypoint들의 위치와 자세를 rviz 위에 Axis로 나타냄
            for (std::size_t i = 0; i < num_waypoints; ++i)
            {
                visual_tools.publishAxisLabeled(waypoints[i], "waypoint" + std::to_string(i));
            }
            // 실제 end-effector의 궤적과 waypoint들을 직선으로 연결한 선 비교를 위해 rviz 위에 직선을 그림
            visual_tools.publishPath(straight_line, rvt::RED, rvt::SMALL);
            // Trigger()를 실행해야 앞에 publish한 내용들이 그려짐
            visual_tools.trigger();

            // Result
            // 탐색 결과인 my_plan에는 걸린 시간정보도 포함되어 있음
            ROS_INFO("Planning successful");
            ROS_INFO("Planning time: %.4f sec", my_plan.planning_time_);
            move_group.execute(my_plan);
            ROS_INFO("Moving successful");

            // Last joints
            // 움직이는 데 사용된 joint_trajectory의 마지막 값 출력
            // Joint waypoint와 값이 비슷한지 비교 가능
            trajectory_msgs::JointTrajectoryPoint last_point = my_plan.trajectory_.joint_trajectory.points.back();
            for (std::size_t i = 0; i < last_point.positions.size(); ++i)
            {
                ROS_INFO("\t(Current) Joint %d: %f", i, last_point.positions[i]);
            }
        }
        else
        {
            ROS_WARN("Planning failed");
        }
            // Indexing
            i = (i + 1) % num_waypoints;
            ROS_INFO("-----------------\n");
        }   
        ros::waitForShutdown();
        return 0;
}
   