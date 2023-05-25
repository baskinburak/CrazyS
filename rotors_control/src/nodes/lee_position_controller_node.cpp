/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lee_position_controller_node.h"

#include <math/Types.h>
#include <mav_msgs/default_topics.h>
#include <mrnav/Converters.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

LeePositionControllerNode::LeePositionControllerNode()
    : sbc_maximum_Linf_acceleration_(2.0), object_sensing_distance_(3.0) {
    InitializeParams();

    ros::NodeHandle nh;

    cmd_pose_sub_ =
        nh.subscribe(mav_msgs::default_topics::COMMAND_POSE, 1,
                     &LeePositionControllerNode::CommandPoseCallback, this);

    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(
        mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
        &LeePositionControllerNode::MultiDofJointTrajectoryCallback, this);

    odometry_sub_ =
        nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                     &LeePositionControllerNode::OdometryCallback, this);

    teammate_shapes_and_states_sub_ = nh.subscribe(
        "teammate_shapes_and_states", 1000,
        &LeePositionControllerNode::TeammateShapesAndStatesCallback, this);

    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
        mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    command_timer_ = nh.createTimer(
        ros::Duration(0), &LeePositionControllerNode::TimedCommandCallback,
        this, true, false);

    static_objects_sub_ =
        nh.subscribe("static_objects", 1000,
                     &LeePositionControllerNode::StaticObjectsCallback, this);

    static_objects_ = std::make_shared<OcTree>();

    dynamic_object_shapes_and_states_sub_ = nh.subscribe(
        "dynamic_object_shapes_and_states", 1000,
        &LeePositionControllerNode::DynamicObjectShapesAndStatesCallback, this);
}

LeePositionControllerNode::~LeePositionControllerNode() {}

void LeePositionControllerNode::InitializeParams() {
    using VectorDIM = math::VectorDIM<double, 3U>;
    using AlignedBox = math::AlignedBox<double, 3U>;

    ros::NodeHandle pnh("~");

    // Read parameters from rosparam.
    GetRosParameter(
        pnh, "position_gain/x",
        lee_position_controller_.controller_parameters_.position_gain_.x(),
        &lee_position_controller_.controller_parameters_.position_gain_.x());
    GetRosParameter(
        pnh, "position_gain/y",
        lee_position_controller_.controller_parameters_.position_gain_.y(),
        &lee_position_controller_.controller_parameters_.position_gain_.y());
    GetRosParameter(
        pnh, "position_gain/z",
        lee_position_controller_.controller_parameters_.position_gain_.z(),
        &lee_position_controller_.controller_parameters_.position_gain_.z());
    GetRosParameter(
        pnh, "velocity_gain/x",
        lee_position_controller_.controller_parameters_.velocity_gain_.x(),
        &lee_position_controller_.controller_parameters_.velocity_gain_.x());
    GetRosParameter(
        pnh, "velocity_gain/y",
        lee_position_controller_.controller_parameters_.velocity_gain_.y(),
        &lee_position_controller_.controller_parameters_.velocity_gain_.y());
    GetRosParameter(
        pnh, "velocity_gain/z",
        lee_position_controller_.controller_parameters_.velocity_gain_.z(),
        &lee_position_controller_.controller_parameters_.velocity_gain_.z());
    GetRosParameter(
        pnh, "attitude_gain/x",
        lee_position_controller_.controller_parameters_.attitude_gain_.x(),
        &lee_position_controller_.controller_parameters_.attitude_gain_.x());
    GetRosParameter(
        pnh, "attitude_gain/y",
        lee_position_controller_.controller_parameters_.attitude_gain_.y(),
        &lee_position_controller_.controller_parameters_.attitude_gain_.y());
    GetRosParameter(
        pnh, "attitude_gain/z",
        lee_position_controller_.controller_parameters_.attitude_gain_.z(),
        &lee_position_controller_.controller_parameters_.attitude_gain_.z());
    GetRosParameter(
        pnh, "angular_rate_gain/x",
        lee_position_controller_.controller_parameters_.angular_rate_gain_.x(),
        &lee_position_controller_.controller_parameters_.angular_rate_gain_
             .x());
    GetRosParameter(
        pnh, "angular_rate_gain/y",
        lee_position_controller_.controller_parameters_.angular_rate_gain_.y(),
        &lee_position_controller_.controller_parameters_.angular_rate_gain_
             .y());
    GetRosParameter(
        pnh, "angular_rate_gain/z",
        lee_position_controller_.controller_parameters_.angular_rate_gain_.z(),
        &lee_position_controller_.controller_parameters_.angular_rate_gain_
             .z());
    GetVehicleParameters(pnh, &lee_position_controller_.vehicle_parameters_);
    lee_position_controller_.InitializeParameters();

    GetRosParameter(pnh, "sbc/maximum_Linf_acceleration", 2.0,
                    &sbc_maximum_Linf_acceleration_);

    double sbc_maximum_Linf_velocity;
    double sbc_aggressiveness;
    double sbc_replanning_period;

    GetRosParameter(pnh, "sbc/maximum_Linf_velocity", 1.0,
                    &sbc_maximum_Linf_velocity);

    GetRosParameter(pnh, "sbc/aggressiveness", 0.2, &sbc_aggressiveness);

    GetRosParameter(pnh, "sbc/replanning_period", 0.1, &sbc_replanning_period);

    ros::NodeHandle nh;

    std::vector<double> collision_shape_min_at_zero{-0.2, -0.2, -0.2},
        collision_shape_max_at_zero{0.2, 0.2, 0.2};
    GetRosParameter(nh, "self_collision_shape/min_at_zero",
                    collision_shape_min_at_zero, &collision_shape_min_at_zero);

    GetRosParameter(nh, "self_collision_shape/max_at_zero",
                    collision_shape_max_at_zero, &collision_shape_max_at_zero);

    const AlignedBox collision_shape_at_zero{
        VectorDIM{collision_shape_min_at_zero[0],
                  collision_shape_min_at_zero[1],
                  collision_shape_min_at_zero[2]},
        VectorDIM{collision_shape_max_at_zero[0],
                  collision_shape_max_at_zero[1],
                  collision_shape_max_at_zero[2]}};

    const double sbc_radius =
        0.5 *
        (collision_shape_at_zero.max() - collision_shape_at_zero.min()).norm();

    bool enable_sbc;
    GetRosParameter(pnh, "sbc/enable", false, &enable_sbc);

    if (enable_sbc) {
        lee_position_controller_.EnableSBC(
            sbc_maximum_Linf_velocity, sbc_maximum_Linf_acceleration_,
            sbc_aggressiveness, sbc_radius, sbc_replanning_period);
    }

    GetRosParameter(pnh, "object_sensing_distance", 3.0,
                    &object_sensing_distance_);

    GetRosParameter(pnh, "static_objects_minimum_existence_probability", 0.1,
                    &static_objects_minimum_existence_probability_);
}
void LeePositionControllerNode::Publish() {}

void LeePositionControllerNode::CommandPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {
    // Clear all pending commands.
    command_timer_.stop();
    commands_.clear();
    command_waiting_times_.clear();

    mav_msgs::EigenTrajectoryPoint eigen_reference;
    mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
    commands_.push_front(eigen_reference);

    lee_position_controller_.SetTrajectoryPoint(commands_.front());
    commands_.pop_front();
}

void LeePositionControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
    // Clear all pending commands.
    command_timer_.stop();
    commands_.clear();
    command_waiting_times_.clear();

    const size_t n_commands = msg->points.size();

    if (n_commands < 1) {
        ROS_WARN_STREAM(
            "Got MultiDOFJointTrajectory message, but message has no points.");
        return;
    }

    mav_msgs::EigenTrajectoryPoint eigen_reference;
    mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(),
                                          &eigen_reference);
    commands_.push_front(eigen_reference);

    for (size_t i = 1; i < n_commands; ++i) {
        const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before =
            msg->points[i - 1];
        const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference =
            msg->points[i];

        mav_msgs::eigenTrajectoryPointFromMsg(current_reference,
                                              &eigen_reference);

        commands_.push_back(eigen_reference);
        command_waiting_times_.push_back(current_reference.time_from_start -
                                         reference_before.time_from_start);
    }

    // We can trigger the first command immediately.
    lee_position_controller_.SetTrajectoryPoint(commands_.front());
    commands_.pop_front();

    if (n_commands > 1) {
        command_timer_.setPeriod(command_waiting_times_.front());
        command_waiting_times_.pop_front();
        command_timer_.start();
    }
}

void LeePositionControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {
    if (commands_.empty()) {
        ROS_WARN("Commands empty, this should not happen here");
        return;
    }

    const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
    lee_position_controller_.SetTrajectoryPoint(commands_.front());
    commands_.pop_front();
    command_timer_.stop();
    if (!command_waiting_times_.empty()) {
        command_timer_.setPeriod(command_waiting_times_.front());
        command_waiting_times_.pop_front();
        command_timer_.start();
    }
}

void LeePositionControllerNode::OdometryCallback(
    const nav_msgs::OdometryConstPtr& odometry_msg) {
    ROS_INFO_ONCE("LeePositionController got first odometry message.");
    using NeighborRobotDescription =
        LeePositionController::NeighborRobotDescription;
    using NeighborObstacleDescription =
        LeePositionController::NeighborObstacleDescription;
    using ObjectState = sbc::NominalSBC<double, 3U>::ObjectState;
    using ProbabilisticObstacle = map::OcTree<double>::ProbabilisticObstacle;
    using AlignedBox = math::AlignedBox<double, 3U>;
    using VectorDIM = math::VectorDIM<double, 3U>;

    EigenOdometry odometry;
    eigenOdometryFromMsg(odometry_msg, &odometry);
    lee_position_controller_.SetOdometry(odometry);

    const double current_time = ros::Time::now().toSec();
    // std::cout << "current_time: " << current_time << std::endl;

    std::vector<NeighborRobotDescription> neighbor_robot_descriptions;

    // Teammates
    for (const std::pair<std::string, double>& teammate_id_and_radius :
         teammate_radii_) {
        const std::string& teammate_id = teammate_id_and_radius.first;
        const double& radius = teammate_id_and_radius.second;
        std::deque<mrnav::TimedState>& teammate_state_sequence =
            teammate_state_sequences_.at(teammate_id);
        const absl::StatusOr<mrnav::TimedState>
            teammate_current_state_status_or =
                mrnav::computeStateAt(teammate_state_sequence, current_time);

        if (!teammate_current_state_status_or.ok()) {
            ROS_WARN_STREAM("Could not compute state for teammate "
                            << teammate_id << " at time " << current_time);
            continue;
        }

        const mrnav::TimedState& teammate_current_state =
            teammate_current_state_status_or.value();

        // std::cout << "last_state / " << teammate_id
        //           << "time: " << teammate_state_sequence.back().time_point
        //           << " position: "
        //           << teammate_state_sequence.back().position.transpose()
        //           << " velocity: "
        //           << teammate_state_sequence.back().velocity.transpose()
        //           << std::endl;

        // std::cout << "computed teammate state / "
        //           << "teammate_id: " << teammate_id << " radius: " << radius
        //           << "time: " << current_time << " position: "
        //           << teammate_current_state.position.transpose()
        //           << " velocity: "
        //           << teammate_current_state.velocity.transpose() <<
        //           std::endl;

        neighbor_robot_descriptions.push_back(NeighborRobotDescription{
            .current_state =
                ObjectState{.position = teammate_current_state.position,
                            .velocity = teammate_current_state.velocity},
            .radius = radius,
            .maximum_Linf_acceleration_lower_bound =
                sbc_maximum_Linf_acceleration_});

        mrnav::clearStateHistory(teammate_state_sequence, current_time);
    }

    std::vector<NeighborObstacleDescription> neighbor_obstacle_descriptions;

    // Static objects
    const AlignedBox query_box{
        odometry.position - VectorDIM::Constant(object_sensing_distance_),
        odometry.position + VectorDIM::Constant(object_sensing_distance_)};

    std::vector<ProbabilisticObstacle> static_objects_in_query_box =
        static_objects_->intersectingProbabilisticObstacles(query_box);

    for (const ProbabilisticObstacle& obstacle : static_objects_in_query_box) {
        if (obstacle.existance_probability_ <
            static_objects_minimum_existence_probability_) {
            continue;
        }

        const AlignedBox& obstacle_box = obstacle.bounding_box_;
        const VectorDIM obstacle_position = obstacle_box.center();
        const VectorDIM obstacle_velocity = VectorDIM::Zero();
        const double obstacle_radius =
            (obstacle_box.max() - obstacle_box.min()).norm() * 0.5;

        neighbor_obstacle_descriptions.push_back(NeighborObstacleDescription{
            .current_state = ObjectState{.position = obstacle_position,
                                         .velocity = obstacle_velocity},
            .radius = obstacle_radius});
    }

    // Dynamic Objects
    for (std::pair<const std::string, std::deque<mrnav::TimedState>>&
             dynamic_object_id_and_state_sequence :
         dynamic_object_state_sequences_) {
        const std::string& dynamic_object_id =
            dynamic_object_id_and_state_sequence.first;
        std::deque<mrnav::TimedState>& dynamic_object_state_sequence =
            dynamic_object_id_and_state_sequence.second;
        const absl::StatusOr<mrnav::TimedState>
            dynamic_object_current_state_status_or = mrnav::computeStateAt(
                dynamic_object_state_sequence, current_time);
        mrnav::clearStateHistory(dynamic_object_state_sequence, current_time);

        if (!dynamic_object_current_state_status_or.ok()) {
            ROS_WARN_STREAM("Could not compute state for dynamic object "
                            << dynamic_object_id << " at time "
                            << current_time);
            continue;
        }

        const mrnav::TimedState& dynamic_object_current_state =
            dynamic_object_current_state_status_or.value();

        if ((odometry.position - dynamic_object_current_state.position).norm() >
            object_sensing_distance_) {
            continue;
        }

        const AlignedBox& box_at_zero =
            dynamic_object_bounding_boxes_.at(dynamic_object_id);

        neighbor_obstacle_descriptions.push_back(NeighborObstacleDescription{
            .current_state =
                ObjectState{.position = dynamic_object_current_state.position,
                            .velocity = dynamic_object_current_state.velocity},
            .radius = (box_at_zero.max() - box_at_zero.min()).norm() * 0.5});

        // std::cout << "dynamic_object_state_sequence.size(): "
        //           << dynamic_object_state_sequence.size() << std::endl;
    }

    // std::sort(neighbor_obstacle_descriptions.begin(),
    //           neighbor_obstacle_descriptions.end(),
    //           [&odometry](const NeighborObstacleDescription& a,
    //                       const NeighborObstacleDescription& b) {
    //               return (a.current_state.position -
    //               odometry.position).norm() <
    //                      (b.current_state.position -
    //                      odometry.position).norm();
    //           });

    // neighbor_obstacle_descriptions.resize(15);

    lee_position_controller_.SetSBCNeighborDescriptions(
        neighbor_robot_descriptions, neighbor_obstacle_descriptions);

    ros::Time before_calculate_time = ros::Time::now();

    // ROS_INFO_STREAM("calculate rotor velocities started" << std::endl);

    Eigen::VectorXd ref_rotor_velocities;
    lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

    ros::Time after_calculate_time = ros::Time::now();
    // ROS_INFO_STREAM("calculate rotor velocities ended" << std::endl);

    // ROS_INFO("calculate rotor velocities took ",
    //          (after_calculate_time - before_calculate_time).toSec(),
    //          " seconds");

    // std::cout << "calculate rotor velocities took "
    //           << (after_calculate_time - before_calculate_time).toSec()
    //           << " seconds" << std::endl;

    // Todo(ffurrer): Do this in the conversions header.
    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

    actuator_msg->angular_velocities.clear();
    for (int i = 0; i < ref_rotor_velocities.size(); i++)
        actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
    actuator_msg->header.stamp = odometry_msg->header.stamp;

    motor_velocity_reference_pub_.publish(actuator_msg);
}

void LeePositionControllerNode::TeammateShapesAndStatesCallback(
    const mrnav::TeammateShapeState::ConstPtr& teammate_shape_and_state) {
    using VectorDIM = math::VectorDIM<double, 3U>;
    using AlignedBox = math::AlignedBox<double, 3U>;

    const std::string& teammate_id = teammate_shape_and_state->robot_id;
    const AlignedBox teammate_box_at_zero{
        mrnav::convertMsgToVectorDIM(teammate_shape_and_state->bbox_min),
        mrnav::convertMsgToVectorDIM(teammate_shape_and_state->bbox_max)};

    const double teammate_radius =
        0.5 * (teammate_box_at_zero.max() - teammate_box_at_zero.min()).norm();

    teammate_radii_[teammate_id] = teammate_radius;

    mrnav::TimedState teammate_state;
    teammate_state.time_point = teammate_shape_and_state->time_point.toSec();
    teammate_state.position =
        mrnav::convertMsgToVectorDIM(teammate_shape_and_state->position);
    teammate_state.velocity =
        mrnav::convertMsgToVectorDIM(teammate_shape_and_state->velocity);

    teammate_state_sequences_[teammate_id].push_back(teammate_state);
};

void LeePositionControllerNode::StaticObjectsCallback(
    const octomap_msgs::Octomap::ConstPtr& static_objects_msg) {
    octomap::AbstractOcTree* tree =
        octomap_msgs::fullMsgToMap(*static_objects_msg);

    if (tree == nullptr) {
        ROS_WARN_STREAM("octree read failed. ");
        return;
    }

    octomap::OcTree* octree_ptr = dynamic_cast<octomap::OcTree*>(tree);
    if (octree_ptr == nullptr) {
        ROS_WARN_STREAM("octree read failed during cast.");
        return;
    }

    static_objects_->octree().setResolution(octree_ptr->getResolution());
    static_objects_->octree().swapContent(*octree_ptr);
    delete octree_ptr;
}

void LeePositionControllerNode::DynamicObjectShapesAndStatesCallback(
    const mrnav::DynamicObjectShapeState::ConstPtr&
        dynamic_object_shape_and_state) {
    using VectorDIM = math::VectorDIM<double, 3U>;
    using AlignedBox = math::AlignedBox<double, 3U>;

    const std::string& dynamic_object_id =
        dynamic_object_shape_and_state->dynamic_object_id;

    const double time_point =
        dynamic_object_shape_and_state->time_point.toSec();

    const AlignedBox dynamic_object_box_at_zero{
        mrnav::convertMsgToVectorDIM(dynamic_object_shape_and_state->bbox_min),
        mrnav::convertMsgToVectorDIM(dynamic_object_shape_and_state->bbox_max)};

    const VectorDIM dynamic_object_position =
        mrnav::convertMsgToVectorDIM(dynamic_object_shape_and_state->position);

    const VectorDIM dynamic_object_velocity =
        mrnav::convertMsgToVectorDIM(dynamic_object_shape_and_state->velocity);

    dynamic_object_state_sequences_[dynamic_object_id].push_back(
        mrnav::TimedState{.time_point = time_point,
                          .position = dynamic_object_position,
                          .velocity = dynamic_object_velocity});
    dynamic_object_bounding_boxes_[dynamic_object_id] =
        dynamic_object_box_at_zero;
}

}  // namespace rotors_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "lee_position_controller_node");

    rotors_control::LeePositionControllerNode lee_position_controller_node;

    ros::spin();

    return 0;
}
