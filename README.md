# hexapod_ik
(ROS-ready) library to use inverse kinematics on our [hexapods]. This library is used in [hexapod_driver].

## Authors

- Author/Maintainer: Dorian Goepp

## Dependencies

- [trac_ik]: Hardware interface for ros_control and the Dynamixels actuators
- orocos_kdl: kinematics and dynamics library (provided with ROS)

## Documentation

### MultipodInverseKinematics
This class is hosted in `multipod_ik.hpp` and holds all the code needed to make direct and inverse kinematics computations for a multi-legged robot (three joints per leg). The API is very simple so we beleive that the commented header file is sufficient as a documentation.

### Test program
We wrote `multipod_ik` to check the code to be used with our hexaforce robot. We chose to keep it as it might be useful for later development of this library.

See [`hexapod_description`](https://github.com/resibots/hexapod_ros/tree/master/hexapod_description) for the URDF files needed to run these tests.

## LICENSE

[CeCILL]

[CeCILL]: http://www.cecill.info/index.en.html
[hexapods]: http://www.resibots.eu/photos.html#robots
[hexapod_driver]: https://github.com/resibots/hexapod_ros/tree/master/hexapod_driver
[trac_ik]: https://bitbucket.org/traclabs/trac_ik