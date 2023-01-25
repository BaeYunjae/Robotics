// 필요한 헤더 파일 include
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <traj_plan/JointInterpolation.h>

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
    ros::init(argc, argv, "fk_node");
    ros::NodeHandle nh("/fk_node");
    ros::AsyncSpinner spinner(4);   // Use 4 threads
    spinner.start();
    static const std::string LOGNAME = "fk_node";

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
    // 로봇이 추종할 joint 경로를 담을 placeholder
    std::vector<std::vector<double>> joint_waypoints;
    // Corresponding Cartesian waypoints
    // 로봇이 추종할 Forward Kinematics 경로를 담을 placeholder
    std::vector<geometry_msgs::Pose> cartesian_waypoints;

    // 입력된 robot 파라미터에 따라 각 placeholder에 다른 값 저장, 의도되지 않은 값이 입력되면 error로 종료
    if (planning_group.compare("puma_560") == 0)
    {
        joint_waypoints = {
            {-0.158621, 1.182338, -0.839760, 0.000001, 0.342578, -0.158622},  // way0
            {-0.070632, 0.628602, 0.316969, 0.000000, 0.945571, -0.070632},   // way1
            {0.485967, 0.628602, 0.316969, 0.000000, 0.945570, 0.485967},     // way2
            {1.017371, 1.182317, -0.839752, 0.000000, 0.342565, 1.017371}     // way3
        };
        geometry_msgs::Pose cartesian_way1;
        cartesian_way1.position.x = 0.3;
        cartesian_way1.position.y = -0.2;
        cartesian_way1.position.z = 0.6;
        tf2::Quaternion target_quat(tf2::Vector3(1.0, 0.0, 0.0), M_PI);
        cartesian_way1.orientation = tf2::toMsg(target_quat);
        cartesian_waypoints.push_back(geometry_msgs::Pose(cartesian_way1));
        cartesian_waypoints.push_back(geometry_msgs::Pose(cartesian_way1));
        cartesian_waypoints.push_back(geometry_msgs::Pose(cartesian_way1));
        cartesian_waypoints.push_back(geometry_msgs::Pose(cartesian_way1));
        cartesian_waypoints[1].position.x += 0.4;
        cartesian_waypoints[2].position.x += 0.4;
        cartesian_waypoints[2].position.y += 0.4;
        cartesian_waypoints[3].position.y += 0.4;
    }
    else if (planning_group.compare("rrr") == 0)
    {
        joint_waypoints = {
            {0.595, -1.951, 1.356},  // way0
            {0.064, -0.565, 0.501},   // way1
            {0.501, -0.565, 0.064},     // way2
            {1.356, -1.951, 0.595}     // way3
        };
        geometry_msgs::Pose cartesian_way1;
        cartesian_way1.position.x = 0.25;
        cartesian_way1.position.y = -0.1;
        cartesian_way1.position.z = 0.02;
        cartesian_waypoints.push_back(geometry_msgs::Pose(cartesian_way1));
        cartesian_waypoints.push_back(geometry_msgs::Pose(cartesian_way1));
        cartesian_waypoints.push_back(geometry_msgs::Pose(cartesian_way1));
        cartesian_waypoints.push_back(geometry_msgs::Pose(cartesian_way1));
        cartesian_waypoints[1].position.x += 0.2;
        cartesian_waypoints[2].position.x += 0.2;
        cartesian_waypoints[2].position.y += 0.2;
        cartesian_waypoints[3].position.y += 0.2;
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Unsupported planning group: " << planning_group);
        return 1;
    }
    // 실수 방지로 joint_waypoints의 요소 개수를 변수로 저장하여 활용
    int num_waypoints = joint_waypoints.size();

    // For waypoint visualization
    // Rviz에 cartesian_waypoints를 닫힌 도형으로 그리기 위해 모든 요소를 straight_line으로 복사
    // 첫번째 요소를 마지막 요소로 한번 더 복사, straight_line 요소를 선으로 이으면 닫힌 도형
    std::vector<geometry_msgs::Pose> straight_line = cartesian_waypoints;
    straight_line.push_back(cartesian_waypoints[0]);

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
    robot_state::RobotStatePtr kinematic_state(move_group.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup(planning_group);

    // Interpolation
    // ROS 서비스 요청만 필요하기 때문에 클라이언트 구현만 다룸
    // <서비스타입>(서비스 이름)
    ros::ServiceClient client = nh.serviceClient<traj_plan::JointInterpolation>(
        "/traj_plan/spline/joint_trajectory_service");
    
    // Placeholder for joint values
    // 반복문으로 waypoint를 차례대로 순회하며 매 iteration마다 '현재의 joint값'과
    // 'end-effector가 다음 waypoint에 도달했을 때의 joint 값'을 spline으로 연결하여 Joint 경로를 구함
    // current_joints 매 iteration마다 갱신
    std::vector<double> current_joints(joint_model_group->getVariableCount());

    // Demo
    int i = 0;
    while (ros::ok())
    {
        ROS_INFO("< Planning to waypoint %d >", i);
        current_joints = move_group.getCurrentJointValues();

        // Forward kinematics validation
        // 로봇이 움직인 후의 end-effector 위치가 joint값을 넣고 
        // FK로 계산된 위치와 동일한 위치인지 확인하기 위해서 Joint group (kinematic_state)에 joint값을 넣고,
        // 계산된 end-effector의 pose (eef_pose)를 터미널에 출력
        const std::string &eef_name = joint_model_group->getLinkModelNames().back();
        kinematic_state->setJointGroupPositions(joint_model_group, joint_waypoints[i]);
        Eigen::Affine3d eef_transformation = kinematic_state->getGlobalLinkTransform(eef_name);
        geometry_msgs::Pose eef_pose = Eigen::toMsg(eef_transformation);
        ROS_INFO_STREAM("EEF pose of waypoint" << i << " : " << eef_pose);

        // Cubic interpolation
        trajectory_msgs::JointTrajectory req_waypoints;
        req_waypoints.joint_names = joint_model_group->getActiveJointModelNames();
        req_waypoints.points.push_back(trajectory_msgs::JointTrajectoryPoint());
        req_waypoints.points.push_back(trajectory_msgs::JointTrajectoryPoint());
        req_waypoints.points[0].positions = current_joints;
        req_waypoints.points[1].positions = joint_waypoints[i];  // target_joints
    
        ROS_INFO("Wait for service: /traj_plan/spline/joint_trajectory_service");
        client.waitForExistence();

        traj_plan::JointInterpolation srv;
        srv.request.waypoints = req_waypoints;
        ROS_INFO("Calling service");
        ros::Time start_time = ros::Time::now();

        for (int attempt = 10; attempt > 0; attempt--)
        {
            if (client.call(srv))
            {
                ROS_INFO("< Successfully interplated to waypoint %d >", i);
                break;
            }
            else if (attempt > 0)
            {
                ROS_ERROR_STREAM("< Failed to interpolate to waypoint "<< i <<" > (now iter) " << attempt);
            }
            else
            {
                return 1;
            }
        }
        ros::Time end_time = ros::Time::now();

        // Motion
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        my_plan.trajectory_.joint_trajectory = srv.response.result;

        // Path visualization
        std::vector<geometry_msgs::Pose> eef_path;
        convert_JointTraj_to_PoseTraj(
            eef_path,
            kinematic_state,
            planning_group,
            my_plan.trajectory_.joint_trajectory);
        // 실제 end-effector 궤적 LIME_GREEN 색상으로 rviz 위에 그림
        visual_tools.publishPath(eef_path, rvt::LIME_GREEN, rvt::SMALL);
        // end-effector가 거쳐갈 waypoint들의 위치와 자세를 rviz 위에 Axis로 나타냄
        for (std::size_t i = 0; i < num_waypoints; ++i)
        {
            visual_tools.publishAxisLabeled(cartesian_waypoints[i], "waypoint" + std::to_string(i));
        }
        // 실제 end-effector의 궤적과 waypoint들을 직선으로 연결한 선 비교를 위해 rviz 위에 직선을 그림
        visual_tools.publishPath(straight_line, rvt::RED, rvt::SMALL);
        // Trigger()를 실행해야 앞에 publish한 내용들이 그려짐
        visual_tools.trigger();

        // Result
        ROS_INFO("Planning successful");
        ROS_INFO("Planning time: %.4f sec", end_time.toSec() - start_time.toSec());
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

        // Indexing
        i = (i + 1) % num_waypoints;
        ROS_INFO("-----------------\n");
    }   
    ros::waitForShutdown();
    return 0;
}
 
