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

#ifndef ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H
#define ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <sbc/NominalSBC.h>

#include "rotors_control/common.h"
#include "rotors_control/parameters.h"

namespace rotors_control {

// Default values for the lee position controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(6, 6, 6);
static const Eigen::Vector3d kDefaultVelocityGain =
    Eigen::Vector3d(4.7, 4.7, 4.7);
static const Eigen::Vector3d kDefaultAttitudeGain =
    Eigen::Vector3d(3, 3, 0.035);
static const Eigen::Vector3d kDefaultAngularRateGain =
    Eigen::Vector3d(0.52, 0.52, 0.025);

class LeePositionControllerParameters {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LeePositionControllerParameters()
        : position_gain_(kDefaultPositionGain),
          velocity_gain_(kDefaultVelocityGain),
          attitude_gain_(kDefaultAttitudeGain),
          angular_rate_gain_(kDefaultAngularRateGain) {
        calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
    }

    Eigen::Matrix4Xd allocation_matrix_;
    Eigen::Vector3d position_gain_;
    Eigen::Vector3d velocity_gain_;
    Eigen::Vector3d attitude_gain_;
    Eigen::Vector3d angular_rate_gain_;
    RotorConfiguration rotor_configuration_;
};

class LeePositionController {
   public:
    using SBC = sbc::NominalSBC<double, 3U>;
    using NeighborRobotDescription = typename SBC::NeighborRobotDescription;
    using NeighborObstacleDescription =
        typename SBC::NeighborObstacleDescription;
    using VectorDIM = math::VectorDIM<double, 3U>;
    using AlignedBox = math::AlignedBox<double, 3U>;

    LeePositionController();
    ~LeePositionController();
    void InitializeParameters();
    void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities);

    /**
     * @brief Enable SBC safety module
     *
     * @param maximum_Linf_velocity Maximum Linf velocity
     * @param maximum_Linf_acceleration Maximum Linf acceleration
     * @param aggressiveness Aggressiveness parameter of SBC
     * @param radius Radius of the ego robot
     * @param replanning_period Replanning period of SBC, i.e., the time period
     * between consecutive calls to SBC
     *
     * @return true if enable operation succeeds, false otherwise
     */
    bool EnableSBC(double maximum_Linf_velocity,
                   double maximum_Linf_acceleration, double aggressiveness,
                   double radius, double replanning_period);

    /**
     * @brief Set neighbor descriptions for the SBC safety module
     *
     * @details Should be updated as the neighbor states change
     *
     * @param neighbor_robot_descriptions Neighbor robot descriptions
     * @param neighbor_obstacle_descriptions Neighbor obstacle descriptions
     */
    void SetSBCNeighborDescriptions(
        const std::vector<NeighborRobotDescription>&
            neighbor_robot_descriptions,
        const std::vector<NeighborObstacleDescription>&
            neighbor_obstacle_descriptions);

    void SetOdometry(const EigenOdometry& odometry);
    void SetTrajectoryPoint(
        const mav_msgs::EigenTrajectoryPoint& command_trajectory);

    LeePositionControllerParameters controller_parameters_;
    VehicleParameters vehicle_parameters_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   private:
    bool initialized_params_;
    bool controller_active_;

    Eigen::Vector3d normalized_attitude_gain_;
    Eigen::Vector3d normalized_angular_rate_gain_;
    Eigen::MatrixX4d angular_acc_to_rotor_velocities_;

    mav_msgs::EigenTrajectoryPoint command_trajectory_;
    EigenOdometry odometry_;

    // Enable sbc module
    bool enable_sbc_;

    // Last safe acceleration computed by SBC
    VectorDIM last_safe_acceleration_;

    // Neighbor robot descriptions
    std::vector<NeighborRobotDescription> neighbor_robot_descriptions_;

    // Neighbor obstacle descriptions
    std::vector<NeighborObstacleDescription> neighbor_obstacle_descriptions_;

    // Nominal SBC module
    std::shared_ptr<SBC> sbc_;

    void ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                  Eigen::Vector3d* angular_acceleration) const;
    void ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) const;
    void MakeAccelerationSafe(Eigen::Vector3d* acceleration);
};
}  // namespace rotors_control

#endif  // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H
