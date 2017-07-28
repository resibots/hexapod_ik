void generate_cartesian_traj_kdl()
{
    // Create paths for a trajectory

    double x = 0.5, y = -3, z = 0;
    KDL::Vector point_start(x, y, z), point_end(x, z, y);
    KDL::Frame frame_start(point_start), frame_end(point_end);

    // Blank interpolation, between two identity rotation
    KDL::RotationalInterpolation_SingleAxis rot_interpolation;
    rot_interpolation.SetStartEnd(KDL::Rotation(), KDL::Rotation());

    // Build a straight path between the two points
    double eqradius = 0.1;
    KDL::Path_Line path(frame_start, frame_end, &rot_interpolation, eqradius, false);
    // KDL::Path_Line path_return(frame_end, frame_start, &rot_interpolation, eqradius, false);

    // Create a velocity profile for a trajectory

    // trajectory has max speed of 1m/s, acceleration of 1m/s², starts at 0s and
    // finished at 1.5s
    double max_vel = 1, max_acc = 1;
    KDL::VelocityProfile_Trap velocity_profile(max_vel, max_acc);
    velocity_profile.SetProfile(0, path.PathLength());

    // Use these to create the trajectory
    KDL::Trajectory_Segment traj_segment(&path, &velocity_profile, false);

    double step = 0.01; // time step
    // Get frames from the trajectory
    std::cout << "time, x, y, z" << std::endl;
    std::cerr << "Total duration: " << traj_segment.Duration() << std::endl;

    std::ofstream trajectory_file("traj.csv");
    for (double time = 0; time < traj_segment.Duration(); time += step) {
        KDL::Vector position = traj_segment.Pos(time).p;
        trajectory_file << time << ", " << position.x() << ", " << position.y() << ", " << position.z() << std::endl;
        std::cout << "\t" << time << " : " << position.x() << "\t" << position.y() << "\t" << position.z() << std::endl;
    }
}