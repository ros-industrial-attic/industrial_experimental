industrial_experimental/IRC_v2
=======================

Experimental implementation of [REP-I0001](https://github.com/ros-industrial/rep/blob/master/rep-I0001.rst)

  - Maintains backward compatibility with v1 (no code changes required)
  - introduces new `industrial_robot_client2` headers and library
  - supports multi-group and variable # of joints
  - requires robot-side driver support for new REP-I0001 messages
  - introduces new `controller_joint_map` ros parameter to configure flexible joint name/index mapping
  - updated `industrial_robot_simulator` to support `controller_map` configs
  - only builds one set of executable nodes, with v2 support
      * most nodes should support configuration by either existing controller_joint_names or new controller_joint_map parameters
      * but v2 nodes only support v2 simple_message comms to/from robot

