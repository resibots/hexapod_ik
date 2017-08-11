#ifndef HEXAPOD_IK_HPP
#define HEXAPOD_IK_HPP

/** This class is a standalone inverse kinematics for a hexapod or something with some.

    standalone means that it does not use ROS. However, it uses a URDF string and
    hence needs a URDF parser.

    TODO: Where relevant, use exceptions rather than status integers

**/
namespace multipod_ik {
    template <NLegs = 6>
    class MultipodInverseKinematics {
    public:
        // deleted methods (to prevent defaults)
        MultipodInverseKinematics(MultipodInverseKinematics&) = 0;
        operator=(MultipodInverseKinematics&) = 0;

        MultipodInverseKinematics(std::string chain_start, std::array<std::string, NLegs> chain_ends, std::string urdf, double timeout, double eps);
        ~MultipodInverseKinematics();

        void init(
            std::string chain_start,
            std::array<std::string, NLegs> chain_ends,
            std::string urdf,
            double timeout,
            double eps);

        std::array<int, NLegs> cartesian_to_joint(
            const std::array<KDL::JntArray, NLegs>& q_init,
            const std::array<KDL::Frame, NLegs>& frame,
            std::array<KDL::JntArray, NLegs>& q_out,
            const KDL::Twist bounds,
            bool stop_on_failure = false);

        std::array<int, NLegs> joint_to_cartesian(
            const KDL::JntArray& q,
            KDL::Frame& frame,
            bool stop_on_failure = false);

        std::array<int, NLegs> neutral_poses(
            std::array<KDL::Frame, NLegs>& frames_out,
            bool stop_on_failure);

    protected:
        bool make_fk_solvers();

        std::array<std::shared_ptr<TRAC_IK::TRAC_IK>, NLegs> tracik_solvers;
        // used for forward kinematics
        // It is initialized only when make_fk_solvers is first called
        std::array<std::shared_ptr<KDL::ChainFkSolverPos_recursive>, NLegs> _fk_solvers;

        KDL::Twist _bounds;
    };
} // namespace multipod_ik

#endif