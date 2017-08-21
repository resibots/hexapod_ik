/********************************************************************************
Copyright (c) 2016, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PointStamped.h>

// Generate a trajectory
#include <kdl/trajectory_composite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/path_line.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>
#include <hexapod_ik/multipod_ik.hpp>

// Former, hexapod-specific, trajectory generation
#include <hexapod_controller/hexapod_controller_cartesian.hpp>

// for round()
#include <cmath>

// for stringstream
#include <sstream>

std::ostream& operator<<(std::ostream& in, const KDL::JntArray& jnt)
{
    in << "[";
    for (int i = 0; i < jnt.rows() - 1; ++i) {
        in << std::setw(10) << jnt(i) << ", ";
    }
    in << std::setw(10) << jnt(jnt.rows() - 1) << "]";

    return in;
}

std::ostream& operator<<(std::ostream& in, const std::vector<KDL::Vector>& vectors)
{
    for (auto vector : vectors) {
        in << vector << std::endl;
    }
    return in;
}

std::vector<KDL::Frame> generate_cartesian_traj(const double t)
{
    static std::array<std::array<double, 3>, 6> scaling = {{{{0.03, 0.03, 0.03}},
        {{0.03, 0.03, 0.03}},
        {{0.03, 0.03, 0.03}},
        {{0.03, 0.03, 0.03}},
        {{0.03, 0.03, 0.03}},
        {{0.03, 0.03, 0.03}}}};
    // std::vector<double> control_params = {{1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5}};
    static std::vector<double> control_params = {{1, 0, 0.5,
        0, 0.25, 0.5,
        0.7, 0.25, 0.5,
        1, 0.5, 0.5,
        0, 0.25, 0.5,
        0.7, 0.75, 0.5,
        1, 0, 0.5,
        0, 0.25, 0.5,
        0.7, 0.25, 0.5,
        1, 0.5, 0.5,
        0, 0.25, 0.5,
        0.7, 0.75, 0.5,
        1, 0, 0.5,
        0, 0.25, 0.5,
        0.7, 0.25, 0.5,
        1, 0.5, 0.5,
        0, 0.25, 0.5,
        0.7, 0.75, 0.5}};

    static hexapod_controller::HexapodControllerCartesian<> controller(control_params, {}, scaling);

    // Vector of doubles representing the coordinates of each leg's tip at time t
    auto pos = controller.pos(t);

    // Convert the output of the controller to a vector of frames
    std::vector<KDL::Frame> feet;
    for (size_t leg = 0; leg < 6; leg++) {
        feet.push_back(KDL::Frame(KDL::Vector(
            pos[leg][0], pos[leg][1], pos[leg][2])));
    }

    return feet;
}

int main(int argc, char** argv)
{
    srand(1);
    ros::init(argc, argv, "hexapod_ik");
    ros::NodeHandle nh("~");

    // Set Logger level to DEBUG
    // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    //     ros::console::notifyLoggerLevelsChanged();
    // }

    static constexpr uint8_t nLegs = 6;

    int num_samples;
    double timeout;
    std::string chain_start, chain_end, urdf_param, urdf_file;

    // ROS parameters
    // --------------

    nh.param("num_samples", num_samples, 1000);
    nh.param("chain_start", chain_start, std::string(""));
    nh.param("chain_end", chain_end, std::string(""));

    if (chain_start == "" || chain_end == "") {
        ROS_FATAL("Missing chain info in launch file");
        exit(-1);
    }

    nh.param("timeout", timeout, 0.005);
    nh.param("urdf_param", urdf_param, std::string("/robot_description"));
    nh.getParam("/robot_description", urdf_file);

    // Initialization of trac_ik
    // -------------------------
    double eps = 1e-5;

    std::array<std::string, nLegs> chain_ends;
    for (size_t i = 0; i < nLegs; ++i) {
        std::stringstream chain_end_full;
        chain_end_full << chain_end << (int)i;
        chain_ends[i] = chain_end_full.str();
    }

    multipod_ik::MultipodInverseKinematics<nLegs> hexapod_ik(
        chain_start, chain_ends, urdf_file, timeout, eps);

    // Testing the trajectories with Rviz visualisation and cartesian generation
    // -------------------------------------------------------------------------
    ROS_INFO("Testing inverse kinematics for a trajectory");

    ros::Publisher joint_states = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000, true);

    sensor_msgs::JointState state;

    state.name = {{"body_leg_0",
        "leg_0_1_2",
        "leg_0_2_3",
        "body_leg_1",
        "leg_1_1_2",
        "leg_1_2_3",
        "body_leg_2",
        "leg_2_1_2",
        "leg_2_2_3",
        "body_leg_3",
        "leg_3_1_2",
        "leg_3_2_3",
        "body_leg_4",
        "leg_4_1_2",
        "leg_4_2_3",
        "body_leg_5",
        "leg_5_1_2",
        "leg_5_2_3"}};

    // sensible initial values
    state.position = std::vector<double>(18, 0);
    state.header.seq = 0;
    state.header.stamp = ros::Time::now();

    // Loop frequency
    ros::Rate r(50); // Hz

    // Counting time from the start of the loop
    ros::Time beginning = ros::Time::now();
    ros::Duration elapsed;

    while (ros::ok()) {
        elapsed = ros::Time::now() - beginning;

        // at this time, the leg's tips should move to these frames
        std::vector<KDL::Frame> frames = generate_cartesian_traj(elapsed.toSec());

        // joint angles computed by inverse kinematics
        // zero-angle default joint angles for each leg
        std::vector<KDL::JntArray> results(6, KDL::JntArray(3));
        // false as soon as inverse kinematic computation failed for one leg
        bool ik_success = true;

        for (uint8_t leg = 0; leg < nLegs; ++leg) {

            KDL::Frame& frame = frames.at(leg);

            KDL::JntArray joint_angles(3);
            joint_angles(1) = 0.2;
            joint_angles(2) = 0.4;
            KDL::Frame reference_pose;
            hexapod_ik.joint_to_cartesian(leg, joint_angles, reference_pose);
            frame.p += reference_pose.p;
            // frame.p += neutral_pose(tracik_solvers[leg]).p;
            KDL::JntArray& result = results.at(leg);

            // ROS_DEBUG_STREAM("Target position for leg " << (int)leg << ": " << frame.p);

            // first reference to "result" is the nominal joint angles
            if (hexapod_ik.cartesian_to_joint(leg, result, frame, result) < 0) {
                ROS_WARN_STREAM("Failed to solve inverse kinematics problem for leg "
                    << (int)leg << ". Target frame was\n"
                    << frame << "\nand result is " << result);
                ik_success = false;
                break;
            }
            else {
                // ROS_DEBUG_STREAM("Found a solution for leg "
                //     << (int)leg << " with joint values:" << result);

                state.position.at(leg * 3) = result(0);
                state.position.at(leg * 3 + 1) = result(1);
                state.position.at(leg * 3 + 2) = result(2);
            }
        }

        // if all the inverse kinematics worked, publish the pose
        if (ik_success) {
            state.header.seq++;
            state.header.stamp = ros::Time::now();
            joint_states.publish(state);
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
