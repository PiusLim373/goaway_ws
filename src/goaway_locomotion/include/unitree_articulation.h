// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "isaaclab/assets/articulation/articulation.h"
#include "Types.h"

namespace unitree
{

template <typename LowStatePtr>
class BaseArticulation : public isaaclab::Articulation
{
public:
    BaseArticulation(LowStatePtr lowstate_, std::shared_ptr<CmdVel_t> cmd_vel_ = nullptr)
    : lowstate(lowstate_), cmd_vel(cmd_vel_)
    {
        data.joystick = &lowstate->joystick;
    }

    void update() override
    {
        if (cmd_vel)
        {
            std::lock_guard<std::mutex> lock(cmd_vel->mutex_);
            data.cmd_vel_fresh = !cmd_vel->isTimeout();
            data.cmd_vel = Eigen::Vector3f(
                cmd_vel->msg_.linear().x(), cmd_vel->msg_.linear().y(), cmd_vel->msg_.angular().z());
        }

        std::lock_guard<std::mutex> lock(lowstate->mutex_);
        // base_angular_velocity
        for(int i(0); i<3; i++) {
            data.root_ang_vel_b[i] = lowstate->msg_.imu_state().gyroscope()[i];
        }
        // project_gravity_body
        data.root_quat_w = Eigen::Quaternionf(
            lowstate->msg_.imu_state().quaternion()[0],
            lowstate->msg_.imu_state().quaternion()[1],
            lowstate->msg_.imu_state().quaternion()[2],
            lowstate->msg_.imu_state().quaternion()[3]
        );
        data.projected_gravity_b = data.root_quat_w.conjugate() * data.GRAVITY_VEC_W;
        // joint positions and velocities
        for(int i(0); i< data.joint_ids_map.size(); i++) {
            data.joint_pos[i] = lowstate->msg_.motor_state()[data.joint_ids_map[i]].q();
            data.joint_vel[i] = lowstate->msg_.motor_state()[data.joint_ids_map[i]].dq();
        }
    }

    LowStatePtr lowstate;
    std::shared_ptr<CmdVel_t> cmd_vel;
};

}