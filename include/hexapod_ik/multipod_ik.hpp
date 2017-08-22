#ifndef HEXAPOD_IK_HPP
#define HEXAPOD_IK_HPP

#include <trac_ik/trac_ik.hpp>

#include <string>
#include <sstream>

/** This class is a standalone inverse kinematics for a hexapod or something with some.

    standalone means that it does not use ROS. However, it uses a URDF string and
    hence needs a URDF parser.

    TODO: Where relevant, use exceptions rather than status integers

    TODO: set/get bounds

**/
namespace multipod_ik {
    template <int NLegs = 6>
    class MultipodInverseKinematics {
    public:
        // deleted methods (to prevent defaults)
        MultipodInverseKinematics(const MultipodInverseKinematics&) = delete;
        MultipodInverseKinematics& operator=(MultipodInverseKinematics&) = delete;

        /** Main constructor of MultipodInverseKinematics.

            This method will instanciate one inverse kinematic solver per leg
            and make them all work only in position (no orientation).

            @param urdf string of the urdf file modeling the multipod
            @param chain_start frame of the model used as the reference
            @param chain_ends one string per leg, naming the frame at the
                extremity of each leg
            @param timeout the amout of time allowed for the inverse kinematics
                to run
            @param eps see the documentation of Track_IK
        **/
        MultipodInverseKinematics(
            std::string chain_start,
            std::array<std::string, NLegs> chain_ends,
            std::string urdf,
            double timeout,
            double eps)
        {
            // Construct the objects doing inverse kinematic computations
            for (size_t leg = 0; leg < NLegs; ++leg) {
                // This constructor parses the URDF loaded in rosparm urdf_param into the
                // needed KDL structures.
                _tracik_solvers[leg] = std::make_shared<TRAC_IK::TRAC_IK>(
                    urdf, chain_start, chain_ends[leg], timeout, eps);
            }

            // Set rotational bounds to max, so that the kinematic ignores the rotation
            // part of the target pose
            _bounds = KDL::Twist::Zero();
            _bounds.rot.x(std::numeric_limits<float>::max());
            _bounds.rot.y(std::numeric_limits<float>::max());
            _bounds.rot.z(std::numeric_limits<float>::max());
        }

        /** Get joint angles from end-effector pose.

            @param q_init joint angles used to initialise the inverse kinematics
            @param frame target poses
            @param q_out resulting joint angles as found by inverse kinematics
            @param stop_on_failure if true, the function will return as soon as
                the inverse kinematic computation fails for one leg (we walk
                through them in ascending leg number).

            @return status of the computation of each leg; failed if < 0
        **/
        std::array<int, NLegs> cartesian_to_joint(
            const std::array<KDL::JntArray, NLegs>& q_init,
            const std::array<KDL::Frame, NLegs>& frame,
            std::array<KDL::JntArray, NLegs>& q_out,
            bool stop_on_failure = false)
        {
            std::array<int, NLegs> status;

            for (size_t leg = 0; leg < NLegs; ++leg) {
                status[leg] = _tracik_solvers[leg]->CartToJnt(q_init[leg], frame[leg], q_out[leg], _bounds);
                if (stop_on_failure && status[leg] < 0)
                    break;
            }

            return status;
        }

        /** Get joint angles from end-effector pose.

            @see cartesian_to_joint(const std::array<KDL::JntArray, NLegs>& q_init,
                const std::array<KDL::Frame, NLegs>& frame,
                std::array<KDL::JntArray, NLegs>& q_out, bool stop_on_failure)
                whereas this method computes for a single leg

            @param leg id of the leg

            @return status of the computation; failed if < 0
        **/
        int cartesian_to_joint(
            size_t leg,
            const KDL::JntArray& q_init,
            const KDL::Frame& frame,
            KDL::JntArray& q_out)
        {
            return _tracik_solvers[leg]->CartToJnt(q_init, frame, q_out, _bounds);
        }

        /** Compute leg tip poses from joint angles.

            @param q joint angles for the legs
            @param frames_out computed poses of each leg
            @param stop_on_failure if true, the function will return as soon as
                the inverse kinematic computation fails for one leg (we walk
                through them in ascending leg number).

            @return status of the computation of each leg; failed if < 0
        **/
        std::array<int, NLegs> joint_to_cartesian(
            const std::array<KDL::JntArray, NLegs>& q,
            std::array<KDL::Frame, NLegs>& frames_out,
            bool stop_on_failure = false)
        {
            std::array<int, NLegs> status(-1);

            // Build the KDL forward kinematics models
            bool initialized = make_fk_solvers();
            if (!initialized)
                return status;

            // Compute the pose of each chain
            for (size_t leg = 0; leg < NLegs; ++leg) {
                status[leg] = _fk_solvers[leg]->JntToCart(q[leg], frames_out[leg]);
                if (stop_on_failure && status[leg] < 0)
                    break;
            }

            return status;
        }

        /** Compute leg tip pose from joint angles.

            @see joint_to_cartesian( const std::array<KDL::JntArray, NLegs>& q,
                std::array<KDL::Frame, NLegs>& frames_out, bool stop_on_failure);
                this one works for a single leg

            @param leg id of the leg
            @param q joint angles for the leg
            @param frame_out computed pose

            @return status of the computation; failed if < 0
        **/
        int joint_to_cartesian(
            size_t leg,
            const KDL::JntArray& q,
            KDL::Frame& frame_out)
        {
            // Build the KDL forward kinematics models
            bool initialized = make_fk_solvers();
            if (!initialized)
                return -1;

            // Compute the pose for the leg
            return _fk_solvers[leg]->JntToCart(q, frame_out);
        }

        /** Give leg tip poses when joint angles are all 0.

            @param frames_out leg poses
            @param stop_on_failure if true, the function will return as soon as
                the inverse kinematic computation fails for one leg (we walk
                through them in ascending leg number).

            @return status of the computation of each leg; failed if < 0
        **/
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

            return joint_to_cartesian(q, frames_out, stop_on_failure);
        }

        /** Give leg tip poses when joint angles are all 0.

            @see neutral_poses(std::array<KDL::Frame, NLegs>& frames_out,
                bool stop_on_failure); this one is for a single leg

            @param leg id of the leg
            @param frame_out leg pose

            @return status of the computation of each leg; failed if < 0
        **/
        int neutral_pose(
            size_t leg,
            KDL::Frame& frame_out)
        {
            // ut all joints in neutral pose, i.e., joint value 0
            KDL::JntArray q(3);
            q(0) = 0;
            q(1) = 0;
            q(2) = 0;

            return joint_to_cartesian(leg, q, frame_out);
        }

    protected:
        bool make_fk_solvers()
        {
            bool initialized = true;

            static bool already_computed = false;

            if (!already_computed) {
                for (size_t leg = 0; leg < NLegs; ++leg) {
                    // Get the KDL chains from URDF
                    KDL::Chain chain;
                    initialized &= _tracik_solvers[leg]->getKDLChain(chain);

                    // Set up KDL forward kinematics
                    _fk_solvers[leg] = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
                }
                already_computed = true;
            }

            return initialized;
        }

        std::array<std::shared_ptr<TRAC_IK::TRAC_IK>, NLegs> _tracik_solvers;
        // used for forward kinematics
        // It is initialized only when make_fk_solvers is first called
        std::array<std::shared_ptr<KDL::ChainFkSolverPos_recursive>, NLegs> _fk_solvers;

        KDL::Twist _bounds;
    };
} // namespace multipod_ik

#endif