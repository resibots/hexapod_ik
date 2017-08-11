#include <hexapod_ik/multipod_ik.hpp>

#include <sstream>

using namespace multipod_ik;

MultipodInverseKinematics::MultipodInverseKinematics(
    std::string chain_start,
    std::array<std::string, NLegs> chain_ends,
    std::string urdf,
    double timeout,
    double eps)
{
    init(chain_start, chain_ends, urdf, timeout, eps);

    // Set rotational bounds to max, so that the kinematic ignores the rotation
    // part of the target pose
    _bounds = KDL::Twist::Zero();
    _bounds.rot.x(std::numeric_limits<float>::max());
    _bounds.rot.y(std::numeric_limits<float>::max());
    _bounds.rot.z(std::numeric_limits<float>::max());
}
MultipodInverseKinematics::~MultipodInverseKinematics();

void MultipodInverseKinematics::init(
    std::string chain_start,
    std::array<std::string, NLegs> chain_ends,
    std::string urdf,
    double timeout,
    double eps)
{
    for (auto chain_end : chain_ends) {
        // This constructor parses the URDF loaded in rosparm urdf_param into the
        // needed KDL structures.
        tracik_solvers[i] = std::make_shared<TRAC_IK::TRAC_IK>(
            urdf, chain_start, chain_end, timeout, eps);
    }
}

std::array<int, NLegs> cartesian_to_joint(
    const std::array<KDL::JntArray, NLegs>& q_init,
    const std::array<KDL::Frame, NLegs>& frame,
    std::array<KDL::JntArray, NLegs>& q_out,
    const KDL::Twist bounds,
    bool stop_on_failure)
{
    std::array<int, NLegs> status;

    for (size_t leg = 0; leg < NLegs; ++leg) {
        status[leg] = tracik_solvers[leg]->CartToJnt(q_init[leg], frame[leg], q_out[leg], bounds);
        if (stop_on_failure && status[leg] < 0)
            break;
    }

    return status;
}

std::array<int, NLegs> joint_to_cartesian(
    const std::array<KDL::JntArray, NLegs>& q,
    std::array<KDL::Frame, NLegs>& frames_out,
    bool stop_on_failure)
{
    std::array<int, NLegs> status(-1);

    // Build the KDL forward kinematics models
    bool initialized = make_fk_solvers();
    if (!initialized)
        return status;

    // Compute the pose of each chain
    for (size_t leg = 0; leg < NLegs; ++leg) {
        status[leg] = fk_solver[leg].JntToCart(q[leg], frame[leg]);
        if (stop_on_failure && status[leg] < 0)
            break;
    }

    return status;
}

std::array<int, NLegs> neutral_poses(
    std::array<KDL::Frame, NLegs>& frames_out,
    bool stop_on_failure)
{
    // ut all joints in neutral pose, i.e., joint value 0
    std::array<KDL::JntArray, NLegs> q(KDL::JntArray(3));
    for (size_t leg = 0; leg < NLegs; ++leg) {
        q[leg](0) = 0;
        q[leg](1) = 0;
        q[leg](2) = 0;
    }

    return joint_to_cartesian(q frames, stop_on_failure);
}

const TRAC_IK::TRAC_IK& MultipodInverseKinematics::trak_ik_solver(size_t leg) const;

bool make_fk_solvers()
{
    bool initialized = true;

    static bool already_computed = false;

    if (!already_computed) {
        for (size_t leg = 0; leg < NLegs; ++leg) {
            // Get the KDL chains from URDF
            KDL::Chain chain;
            initialized &= tracik_solver->getKDLChain(chain);

            // Set up KDL forward kinematics
            _fk_solvers[leg] = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
        }
        already_computed = true;
    }

    return initialized;
}