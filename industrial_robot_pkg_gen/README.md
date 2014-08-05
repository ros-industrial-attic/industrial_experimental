This package is an experimental package to help automate the creation of ROS-Industrial robot packages.  This package incorporates the following standard procedures:

Standardize URDFs – see http://wiki.ros.org/Industrial/Tutorials/Create%20a%20URDF%20for%20an%20Industrial%20Robot

Standardize directory structure – see http://wiki.ros.org/Industrial/Tutorials/SuggestedPackageLayoutNewRepositories

Standardize moveit config packages – see http://wiki.ros.org/Industrial/Tutorials/Create_a_MoveIt_Pkg_for_an_Industrial_Robot

Detailed help can be found by running ```rosrun industrial_robot_pkg_gen package_generator --help```.  Subcommand help can be found by running: ```rosrun industrial_robot_pkg_gen package_generator support --help``` or ```rosrun industrial_robot_pkg_gen package_generator moveit --help```

Here is the suggested work flow:

1.	Create a URDF
1.	Run: ```rosrun industrial_robot_pkg_gen package_generator support``` with associated parameters.
1.	Fill in model specific information in the generated package:
 1.	collision/visual meshes
 1.	urdf and macros (should match naming convention of other packages)
1.	Run: ```rosrun industrial_robot_pkg_gen package_generator moveit``` with associated parameters.
1.	Fill in model specific parameters for the moveit config
 1.	Check joint_limits.yaml for proper values
