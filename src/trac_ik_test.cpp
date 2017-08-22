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

// Trac_ik and KDL solver
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

// Generate a trajectory using KDL
#include <kdl/trajectory_composite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/path_line.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PointStamped.h>

// Hexapod-specific trajectory generation
#include <hexapod_controller/hexapod_controller_cartesian.hpp>

#include <cmath> // for round()
#include <sstream> // stringstream

double fRand(double min, double max)
{
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
}

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

KDL::Frame joint_to_cartesian(std::shared_ptr<TRAC_IK::TRAC_IK> tracik_solver, KDL::JntArray joint_angles)
{

    // Get the KDL chain from URDF
    KDL::Chain chain;
    bool valid = tracik_solver->getKDLChain(chain);

    // Set up KDL forward kinematics to create goal end effector pose
    KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver

    KDL::Frame frame;

    int status = fk_solver.JntToCart(joint_angles, frame);
    if (status < 0) {
        ROS_ERROR_STREAM("Something went bad with the forward kinematics.");
    }
    return frame;
}

/** Get frame the end of a chain when all joints are at zero angle.

    @param tracik_solver shared pointer to inverse kinematics solver

    @return frame at the end of the chain
**/
KDL::Frame neutral_pose(std::shared_ptr<TRAC_IK::TRAC_IK> tracik_solver)
{
    // Get the KDL chain from URDF
    KDL::Chain chain;
    bool valid = tracik_solver->getKDLChain(chain);
    //
    // // Set up KDL forward kinematics to create goal end effector pose
    // KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
    //
    // Set joint angles to zero
    KDL::JntArray joint_angles(chain.getNrOfJoints()); // set to zero at construction
    //
    // KDL::Frame frame;
    //
    // int status = fk_solver.JntToCart(joint_angles, frame);
    // if (status < 0) {
    //     ROS_ERROR_STREAM("Something went bad with the forward kinematics.");
    // }
    // return frame;
    return joint_to_cartesian(tracik_solver, joint_angles);
}

bool test_fk(std::shared_ptr<TRAC_IK::TRAC_IK> tracik_solver)
{
    KDL::JntArray joint_angles(3);
    joint_angles(0) = 0;
    joint_angles(1) = 0;
    joint_angles(2) = 0;

    KDL::Frame frame = joint_to_cartesian(tracik_solver, joint_angles);

    KDL::Vector point = frame.p;
    KDL::Vector end_effector_pose(-0.173711, 0.233132, -0.115);
    return (frame.p == end_effector_pose);
}

void test_ik(
    double num_samples, std::string chain_start, std::string chain_end,
    double timeout, std::string xml_string)
{
    double eps = 1e-5;

    // This constructor parses the URDF loaded in rosparm urdf_param into the
    // needed KDL structures.  We then pull these out to compare against the KDL
    // IK solver.
    TRAC_IK::TRAC_IK tracik_solver(xml_string, chain_start, chain_end, timeout, eps);

    KDL::Chain chain;
    KDL::JntArray ll, ul; //lower joint limits, upper joint limits

    bool valid = tracik_solver.getKDLChain(chain);

    if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
        return;
    }

    valid = tracik_solver.getKDLLimits(ll, ul);

    if (!valid) {
        ROS_ERROR("There were no valid KDL joint limits found");
        return;
    }

    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());

    ROS_INFO("Using %d joints", chain.getNrOfJoints());

    // Set up KDL forward kinematics to create goal end effector pose
    KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver

    // Create Nominal chain configuration midway between all joint limits
    KDL::JntArray nominal(chain.getNrOfJoints());

    for (uint j = 0; j < nominal.data.size(); j++) {
        nominal(j) = (ll(j) + ul(j)) / 2.0;
    }

    // Create desired number of valid, random joint configurations
    std::vector<KDL::JntArray> JointList;
    KDL::JntArray q(chain.getNrOfJoints());

    for (uint i = 0; i < num_samples; i++) {
        for (uint j = 0; j < ll.data.size(); j++) {
            q(j) = fRand(ll(j), ul(j));
        }
        JointList.push_back(q);
    }

    boost::posix_time::ptime start_time;
    boost::posix_time::time_duration diff;

    KDL::JntArray result;
    KDL::Frame end_effector_pose;
    int rc;

    double total_time = 0;
    uint success = 0;
    //
    // ROS_INFO_STREAM("*** Testing KDL with " << num_samples << " random samples");
    //
    // for (uint i = 0; i < num_samples; i++) {
    //     fk_solver.JntToCart(JointList[i], end_effector_pose);
    //     double elapsed = 0;
    //     result = nominal; // start with nominal
    //     start_time = boost::posix_time::microsec_clock::local_time();
    //     do {
    //         q = result; // when iterating start with last solution
    //         rc = kdl_solver.CartToJnt(q, end_effector_pose, result);
    //         diff = boost::posix_time::microsec_clock::local_time() - start_time;
    //         elapsed = diff.total_nanoseconds() / 1e9;
    //     } while (rc < 0 && elapsed < timeout);
    //     total_time += elapsed;
    //     if (rc >= 0)
    //         success++;
    //
    //     if (int((double)i / num_samples * 100) % 10 == 0)
    //         ROS_INFO_STREAM_THROTTLE(1, int((i) / num_samples * 100) << "\% done");
    // }
    //
    // ROS_INFO_STREAM("KDL found " << success << " solutions (" << 100.0 * success / num_samples << "\%) with an average of " << total_time / num_samples << " secs per sample");
    //
    // total_time = 0;
    // success = 0;

    ROS_INFO_STREAM("*** Testing TRAC-IK with " << num_samples << " random samples");

    for (uint i = 0; i < num_samples; i++) {
        fk_solver.JntToCart(JointList[i], end_effector_pose);
        double elapsed = 0;
        start_time = boost::posix_time::microsec_clock::local_time();
        rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
        diff = boost::posix_time::microsec_clock::local_time() - start_time;
        elapsed = diff.total_nanoseconds() / 1e9;
        total_time += elapsed;
        if (rc >= 0)
            success++;

        if (int((double)i / num_samples * 100) % 10 == 0)
            ROS_INFO_STREAM_THROTTLE(1, int((i) / num_samples * 100) << "\% done");
    }

    ROS_INFO_STREAM("TRAC-IK found "
        << success << " solutions (" << 100.0 * success / num_samples
        << "\%) with an average of " << total_time / num_samples
        << " secs per sample");
}

std::vector<KDL::Frame> generate_cartesian_traj(const double t)
{
    static std::array<std::array<double, 3>, 6> scaling = {{{{0.03, 0.03, 0.03}},
        {{0.03, 0.03, 0.03}},
        {{0.03, 0.03, 0.03}},
        {{0.03, 0.03, 0.03}},
        {{0.03, 0.03, 0.03}},
        {{0.03, 0.03, 0.03}}}};

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

std::vector<KDL::JntArray> generate_joint_traj(
    std::shared_ptr<TRAC_IK::TRAC_IK> tracik_solver,
    const std::vector<KDL::Frame>& frames,
    const std::vector<KDL::JntArray>& nominals)
{

    // Set rotational bounds to max, so that the kinematics ignores the rotation
    // part of the target pose
    KDL::Twist bounds = KDL::Twist::Zero();
    bounds.rot.x(std::numeric_limits<float>::max());
    bounds.rot.y(std::numeric_limits<float>::max());
    bounds.rot.z(std::numeric_limits<float>::max());

    assert(frames.size() == nominals.size());

    std::vector<KDL::JntArray> results;
    KDL::JntArray result;

    for (size_t i = 0; i < frames.size(); ++i) {
        std::cout << "Nominal joint angles: " << nominals.at(i) << std::endl;
        std::cout << "Target position: " << frames.at(i).p << std::endl;
        if (!tracik_solver->CartToJnt(nominals.at(i), frames.at(i), result, bounds))
            std::cerr << "Failed to solve inverse kinematics problem" << std::endl;
        else
            results.push_back(result);

        std::cout << "Joint coordinates: " << result << std::endl;
        std::cout << "Related position:  "
                  << joint_to_cartesian(tracik_solver, result).p
                  << std::endl
                  << std::endl;
        // break;
    }

    return results;
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
    std::array<std::shared_ptr<TRAC_IK::TRAC_IK>, 6> tracik_solvers;
    for (size_t i = 0; i < 6; ++i) {
        std::stringstream chain_end_full;
        chain_end_full << chain_end << (int)i;
        ROS_DEBUG_STREAM("Chain end: " << chain_end_full.str());
        // This constructor parses the URDF loaded in rosparm urdf_param into the
        // needed KDL structures.
        tracik_solvers[i] = std::make_shared<TRAC_IK::TRAC_IK>(
            urdf_file, chain_start, chain_end_full.str(), timeout, eps);
    }

    // Test Zone
    // ---------

    if (num_samples < 1)
        num_samples = 1;

    // test_ik(num_samples, chain_start, chain_end, timeout, urdf_file);
    // generate_cartesian_traj_kdl();

    // Convert from joint space to task/cartesian space
    // ------------------------------------------------

    // KDL::JntArray joint_angles(3);
    // joint_angles(0) = 0;
    // joint_angles(1) = 0;
    // joint_angles(2) = 0;
    //
    // KDL::Frame frame = joint_to_cartesian(tracik_solvers[0], joint_angles);
    // std::cout << "Position of the end effector: " << frame.p << std::endl;

    // Test neutral_pose against joint_to_cartesian
    // --------------------------------------------

    // {
    //     KDL::Frame neutral_frame = neutral_pose(tracik_solvers[0]);
    //     std::cout << "Same frames : " << (frame == neutral_frame ? "yes" : "no") << std::endl;
    //     std::cout << "Neutral frame : " << neutral_frame.p << std::endl;
    // }

    // Test the conversion from joint space to task/cartesian space
    // ------------------------------------------------------------

    std::cout << "Testing tracik solver" << std::endl;
    if (test_fk(tracik_solvers[0]))
        std::cout << "Success !" << std::endl;
    else {
        std::cout << "Failure !" << std::endl;
        return -1;
    }

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

    // Set rotational bounds to max, so that the kinematics ignores the rotation
    // part of the target pose
    KDL::Twist tmp_bounds = KDL::Twist::Zero();
    tmp_bounds.rot.x(std::numeric_limits<float>::max());
    tmp_bounds.rot.y(std::numeric_limits<float>::max());
    tmp_bounds.rot.z(std::numeric_limits<float>::max());
    const KDL::Twist bounds(tmp_bounds);
    ROS_DEBUG_STREAM("Tolerances for position and rotation: " << bounds);

    // Counting time from start of the loop
    ros::Time beginning = ros::Time::now();
    ros::Duration elapsed;

    // Loop frequency set to 1Hz
    ros::Rate r(50);

    while (ros::ok()) {
        elapsed = ros::Time::now() - beginning;

        // at this time, the leg's tips should move to these frames
        std::vector<KDL::Frame> frames = generate_cartesian_traj(elapsed.toSec());

        // joint angles computed by inverse kinematics
        // zero-angle default joint angles for each leg
        std::vector<KDL::JntArray> results(6, KDL::JntArray(3));
        // false as soon as inverse kinematic computation failed for one leg
        bool ik_success = true;

        for (uint8_t leg = 0; leg < 6; ++leg) {

            KDL::Frame& frame = frames.at(leg);
            KDL::JntArray joint_angles(3);
            joint_angles(1) = 0.2;
            joint_angles(2) = 0.4;
            frame.p += joint_to_cartesian(tracik_solvers[leg], joint_angles).p;
            // frame.p += neutral_pose(tracik_solvers[leg]).p;
            KDL::JntArray& result = results.at(leg);

            // ROS_DEBUG_STREAM("Target position for leg " << (int)leg << ": " << frame.p);

            // first reference to "result" is the nominal joint angles
            if (tracik_solvers[leg]->CartToJnt(result, frame, result, bounds) < 0) {
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
        // else
        //     ros::Duration(3).sleep();

        ros::spinOnce();
        r.sleep();
    }

    // std::vector<KDL::JntArray> angles = generate_joint_traj(
    //     tracik_solvers[0],
    //     frames,
    //     nominals);

    return 0;
}
