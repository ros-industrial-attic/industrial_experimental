/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "simple_message/simple_message.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/socket/tcp_server.h"
#include "simple_message/messages/joint_message.h"
#include "simple_message/joint_data.h"
#include "simple_message/joint_traj_pt.h"
#include "simple_message/messages/joint_traj_pt_message.h"
#include "simple_message/typed_message.h"
#include "simple_message/joint_traj.h"
#include "simple_message/robot_status.h"
#include "simple_message/messages/robot_status_message.h"
#include "simple_message/messages/dynamic_joint_pt_message.h"
#include "simple_message/messages/dynamic_joint_state_message.h"
#include "simple_message/messages/dynamic_group_status_message.h"

#include <gtest/gtest.h>

using namespace industrial::simple_message;
using namespace industrial::tcp_socket;
using namespace industrial::tcp_client;
using namespace industrial::tcp_server;
using namespace industrial::shared_types;
using namespace industrial::joint_data;
using namespace industrial::joint_message;
using namespace industrial::joint_traj_pt;
using namespace industrial::joint_traj_pt_message;
using namespace industrial::typed_message;
using namespace industrial::joint_traj;
using namespace industrial::robot_status;
using namespace industrial::robot_status_message;

// Message passing routine, used to send and receive a typed message
// Useful for checking the packing and unpacking of message data.
void messagePassing(TypedMessage &send, TypedMessage &recv)
{
  const int tcpPort = 11010;
  char ipAddr[] = "127.0.0.1";

  TcpClient tcpClient;
  TcpServer tcpServer;
  SimpleMessage msgSend, msgRecv;

  ASSERT_TRUE(send.toTopic(msgSend));

  // Construct server

  ASSERT_TRUE(tcpServer.init(tcpPort));

  // Construct a client
  ASSERT_TRUE(tcpClient.init(&ipAddr[0], tcpPort));
  ASSERT_TRUE(tcpClient.makeConnect());

  ASSERT_TRUE(tcpServer.makeConnect());

  ASSERT_TRUE(tcpClient.sendMsg(msgSend));
  ASSERT_TRUE(tcpServer.receiveMsg(msgRecv));
  ASSERT_TRUE(recv.init(msgRecv));
}

TEST(JointMessage, init)
{
  JointData joint;

  joint.init();
  EXPECT_TRUE(joint.setJoint(0, 1.0));
  EXPECT_TRUE(joint.setJoint(1, 2.0));
  EXPECT_TRUE(joint.setJoint(2, 3.0));
  EXPECT_TRUE(joint.setJoint(3, 4.0));
  EXPECT_TRUE(joint.setJoint(4, 5.0));
  EXPECT_TRUE(joint.setJoint(5, 6.0));
  EXPECT_TRUE(joint.setJoint(6, 7.0));
  EXPECT_TRUE(joint.setJoint(7, 8.0));
  EXPECT_TRUE(joint.setJoint(8, 9.0));
  EXPECT_TRUE(joint.setJoint(9, 10.0));

  EXPECT_FALSE(joint.setJoint(10, 11.0));

}

TEST(JointMessage, equal)
{
  JointData jointlhs, jointrhs;

  jointrhs.init();
  jointlhs.init();
  jointlhs.setJoint(0, -1.0);
  jointlhs.setJoint(9, 1.0);

  EXPECT_FALSE(jointlhs==jointrhs);

  jointrhs.setJoint(0, -1.0);
  jointrhs.setJoint(9, 1.0);

  EXPECT_TRUE(jointlhs==jointrhs);

}

TEST(JointMessage, toMessage)
{
  JointData toMessage, fromMessage;
  JointMessage msg;

  toMessage.init();
  toMessage.setJoint(4, 44.44);

  msg.init(1, toMessage);

  fromMessage.copyFrom(msg.getJoints());

  EXPECT_TRUE(toMessage==fromMessage);

}

TEST(JointMessage, Comms)
{

  JointMessage jointSend, jointRecv;
  JointData posSend, posRecv;

  posSend.init();
  posSend.setJoint(0, 1.0);
  posSend.setJoint(1, 2.0);
  posSend.setJoint(2, 3.0);
  posSend.setJoint(3, 4.0);
  posSend.setJoint(4, 5.0);
  posSend.setJoint(5, 6.0);
  posSend.setJoint(6, 7.0);
  posSend.setJoint(7, 8.0);
  posSend.setJoint(8, 9.0);
  posSend.setJoint(9, 10.0);

  jointSend.init(1, posSend);

  messagePassing(jointSend, jointRecv);

  posRecv.copyFrom(jointRecv.getJoints());
  ASSERT_TRUE(posRecv==posSend);
}

TEST(JointTrajPt, equal)
{
  JointTrajPt lhs, rhs;
  JointData joint;

  joint.init();
  ASSERT_TRUE(joint.setJoint(0, 1.0));
  ASSERT_TRUE(joint.setJoint(1, 2.0));
  ASSERT_TRUE(joint.setJoint(2, 3.0));
  ASSERT_TRUE(joint.setJoint(3, 4.0));
  ASSERT_TRUE(joint.setJoint(4, 5.0));
  ASSERT_TRUE(joint.setJoint(5, 6.0));
  ASSERT_TRUE(joint.setJoint(6, 7.0));
  ASSERT_TRUE(joint.setJoint(7, 8.0));
  ASSERT_TRUE(joint.setJoint(8, 9.0));
  ASSERT_TRUE(joint.setJoint(9, 10.0));

  rhs.init(1.0, joint, 50.0, 100);
  EXPECT_FALSE(lhs==rhs);

  lhs.init(0, joint, 0, 0);
  EXPECT_FALSE(lhs==rhs);

  lhs.copyFrom(rhs);
  EXPECT_TRUE(lhs==rhs);

}

TEST(JointTrajPt, toMessage)
{
  JointTrajPt toMessage, fromMessage;
  JointTrajPtMessage msg;

  toMessage.init();
  toMessage.setSequence(99);
  msg.init(toMessage);

  fromMessage.copyFrom(msg.point_);

  EXPECT_TRUE(toMessage==fromMessage);

}

TEST(JointTrajPt, Comms)
{

  JointTrajPtMessage jointSend, jointRecv;
  JointData data;
  JointTrajPt posSend, posRecv;

  data.init();
  data.setJoint(0, 1.0);
  data.setJoint(1, 2.0);
  data.setJoint(2, 3.0);
  data.setJoint(3, 4.0);
  data.setJoint(4, 5.0);
  data.setJoint(5, 6.0);
  data.setJoint(6, 7.0);
  data.setJoint(7, 8.0);
  data.setJoint(8, 9.0);
  data.setJoint(9, 10.0);
  posSend.init(1, data, 99, 100);

  jointSend.init(posSend);

  messagePassing(jointSend, jointRecv);

  posRecv.copyFrom(jointRecv.point_);
  ASSERT_TRUE(posRecv==posSend);
}

TEST(JointTraj, equal)
{
  JointTraj lhs, rhs;
  JointData joint;
  JointTrajPt point;

  joint.init();
  ASSERT_TRUE(joint.setJoint(0, 1.0));
  ASSERT_TRUE(joint.setJoint(1, 2.0));
  ASSERT_TRUE(joint.setJoint(2, 3.0));
  ASSERT_TRUE(joint.setJoint(3, 4.0));
  ASSERT_TRUE(joint.setJoint(4, 5.0));
  ASSERT_TRUE(joint.setJoint(5, 6.0));
  ASSERT_TRUE(joint.setJoint(6, 7.0));
  ASSERT_TRUE(joint.setJoint(7, 8.0));
  ASSERT_TRUE(joint.setJoint(8, 9.0));
  ASSERT_TRUE(joint.setJoint(9, 10.0));

  point.init(1.0, joint, 50.0, 100);
  rhs.addPoint(point);
  EXPECT_FALSE(lhs==rhs);

  lhs.addPoint(point);
  EXPECT_TRUE(lhs==rhs);

  lhs.addPoint(point);
  EXPECT_FALSE(lhs==rhs);

  lhs.copyFrom(rhs);
  EXPECT_TRUE(lhs==rhs);

}

TEST(RobotStatus, enumerations)
{
  // Verifying the disabled state and aliases match
  EXPECT_EQ(TriStates::TS_DISABLED, TriStates::TS_FALSE);
  EXPECT_EQ(TriStates::TS_DISABLED, TriStates::TS_LOW);
  EXPECT_EQ(TriStates::TS_DISABLED, TriStates::TS_OFF);

  // Verifying the enabled state and aliases values match
  EXPECT_EQ(TriStates::TS_ENABLED, TriStates::TS_TRUE);
  EXPECT_EQ(TriStates::TS_ENABLED, TriStates::TS_HIGH);
  EXPECT_EQ(TriStates::TS_ENABLED, TriStates::TS_ON);

  // Verifying the unknown values match (this isn't reqd, but makes sense)
  EXPECT_EQ(TriStates::TS_UNKNOWN, RobotModes::UNKNOWN);
}

TEST(RobotStatus, init)
{
  RobotStatus status;
  RobotStatus empty;
  status.init();
  // An empty (non-initted) status should be initialized in the constructor.
  EXPECT_TRUE(status==empty);
  EXPECT_EQ(status.getDrivesPowered(), TriStates::TS_UNKNOWN);
  EXPECT_EQ(status.getEStopped(), TriStates::TS_UNKNOWN);
  EXPECT_EQ(status.getErrorCode(), 0);
  EXPECT_EQ(status.getInError(), TriStates::TS_UNKNOWN);
  EXPECT_EQ(status.getInMotion(), TriStates::TS_UNKNOWN);
  EXPECT_EQ(status.getMode(), RobotModes::UNKNOWN);
  EXPECT_EQ(status.getMotionPossible(), TriStates::TS_UNKNOWN);
}

TEST(RobotStatus, equal)
{
  RobotStatus lhs, rhs;

  EXPECT_TRUE(lhs==rhs);
  lhs.setDrivesPowered(TriStates::TS_ENABLED);
  EXPECT_FALSE(lhs==rhs);
  rhs.setDrivesPowered(TriStates::TS_ENABLED);
  EXPECT_TRUE(lhs==rhs);

  lhs.setEStopped(TriStates::TS_ENABLED);
  EXPECT_FALSE(lhs==rhs);
  rhs.setEStopped(TriStates::TS_ENABLED);
  EXPECT_TRUE(lhs==rhs);

  lhs.setErrorCode(99);
  EXPECT_FALSE(lhs==rhs);
  rhs.setErrorCode(99);
  EXPECT_TRUE(lhs==rhs);

  lhs.setInError(TriStates::TS_ENABLED);
  EXPECT_FALSE(lhs==rhs);
  rhs.setInError(TriStates::TS_ENABLED);
  EXPECT_TRUE(lhs==rhs);

  lhs.setInMotion(TriStates::TS_ENABLED);
  EXPECT_FALSE(lhs==rhs);
  rhs.setInMotion(TriStates::TS_ENABLED);
  EXPECT_TRUE(lhs==rhs);

  lhs.setMode(RobotModes::AUTO);
  EXPECT_FALSE(lhs==rhs);
  rhs.setMode(RobotModes::AUTO);
  EXPECT_TRUE(lhs==rhs);

  lhs.setMotionPossible(TriStates::TS_ENABLED);
  EXPECT_FALSE(lhs==rhs);
  rhs.setMotionPossible(TriStates::TS_ENABLED);
  EXPECT_TRUE(lhs==rhs);

}

TEST(RobotStatus, toMessage)
{
  RobotStatus toMessage, fromMessage;
  RobotStatusMessage msg;

  toMessage.init(TriStates::TS_ENABLED, TriStates::TS_FALSE, 99, TriStates::TS_TRUE, TriStates::TS_TRUE, RobotModes::MANUAL,
                 TriStates::TS_DISABLED);
  msg.init(toMessage);

  fromMessage.copyFrom(msg.status_);

  EXPECT_TRUE(toMessage==fromMessage);

}

TEST(RobotStatus, Comms)
{
  RobotStatusMessage statusMsgSend, statusMsgRecv;
  RobotStatus statusSend, statusRecv;

  statusSend.init(TriStates::TS_ENABLED, TriStates::TS_FALSE, 99, TriStates::TS_TRUE, TriStates::TS_TRUE, RobotModes::MANUAL,
                   TriStates::TS_DISABLED);

  statusMsgSend.init(statusSend);

  messagePassing(statusMsgSend, statusMsgRecv);

  statusRecv.copyFrom(statusMsgRecv.status_);
  ASSERT_TRUE(statusRecv==statusSend);
}

TEST(DynamicJointPt, grpInit)
{
  using namespace industrial::dynamic_joint_pt;

  DynamicJointPtGrp grp;
  DynamicJointPtGrp emptyGrp;
  shared_real tmpVal;
  std::vector<shared_real> tmpArray;

  grp.init(0);
  EXPECT_TRUE(grp==emptyGrp);  // constructor should also empty-initialize
  EXPECT_EQ(0, grp.getGroupID());
  EXPECT_EQ(0, grp.getNumJoints());
  EXPECT_FALSE(grp.getTime(tmpVal));
  EXPECT_FALSE(grp.getPositions(tmpArray));
  EXPECT_FALSE(grp.getVelocities(tmpArray));
  EXPECT_FALSE(grp.getAccelerations(tmpArray));
  EXPECT_FALSE(grp.getEfforts(tmpArray));
  EXPECT_EQ(12, grp.byteLength());

  grp.init(6);
  EXPECT_FALSE(grp==emptyGrp);
  EXPECT_EQ(6, grp.getNumJoints());
  EXPECT_FALSE(grp.getTime(tmpVal));
  EXPECT_FALSE(grp.getPositions(tmpArray));
  EXPECT_FALSE(grp.getVelocities(tmpArray));
  EXPECT_FALSE(grp.getAccelerations(tmpArray));
  EXPECT_FALSE(grp.getEfforts(tmpArray));
}

TEST(DynamicJointPt, grpAssignment)
{
  using namespace industrial::dynamic_joint_pt;

  const int num_joints = 3;
  DynamicJointPtGrp grp, grp2;
  shared_real tmpVal, refVal;
  std::vector<shared_real> tmpArray, refArray, shortArray;

  refVal = 3.14159;
  for (int i=0; i<num_joints; ++i)
    refArray.push_back(i);
  shortArray = refArray; shortArray.pop_back();

  // test basic assignment expectations
  grp.init(num_joints);
  EXPECT_EQ(grp.getNumJoints(), num_joints);  // init should set num_joints
  EXPECT_TRUE(grp.setGroupID(2));
  EXPECT_EQ(2, grp.getGroupID());
  EXPECT_FALSE(grp.setTime(-1));              // don't allow set invalid value
  EXPECT_TRUE(grp.setTime(refVal));           // allow set valid value
  EXPECT_TRUE(grp.getTime(tmpVal));           // allow read valid value
  EXPECT_EQ(refVal, tmpVal);                  // check write/read values match
  EXPECT_FALSE(grp.setPositions(shortArray));
  EXPECT_TRUE(grp.setPositions(refArray));
  EXPECT_TRUE(grp.getPositions(tmpArray));
  EXPECT_EQ(refArray, tmpArray);
  EXPECT_FALSE(grp.setVelocities(shortArray));
  EXPECT_TRUE(grp.setVelocities(refArray));
  EXPECT_TRUE(grp.setVelocities(tmpArray));
  EXPECT_EQ(refArray, tmpArray);
  EXPECT_FALSE(grp.setAccelerations(shortArray));
  EXPECT_TRUE(grp.setAccelerations(refArray));
  EXPECT_TRUE(grp.setAccelerations(tmpArray));
  EXPECT_EQ(refArray, tmpArray);
  EXPECT_FALSE(grp.setEfforts(shortArray));
  EXPECT_TRUE(grp.setEfforts(refArray));
  EXPECT_TRUE(grp.setEfforts(tmpArray));
  EXPECT_EQ(refArray, tmpArray);
  #ifndef FLOAT64
  EXPECT_EQ(16 + 16*num_joints, grp.byteLength());  // expected length (full msg)
  #else
  EXPECT_EQ(20 + 32*num_joints, grp.byteLength());  // expected length (full msg)
  #endif

  // test the ability to clear data arrays
  grp.clearTime();
  EXPECT_FALSE(grp.isValid(FieldTypes::TIME));
  grp.clearPositions();                             // clear data array
  EXPECT_FALSE(grp.isValid(FieldTypes::POSITION));  // should be marked invalid
  grp.clearVelocities();
  EXPECT_FALSE(grp.isValid(FieldTypes::VELOCITY));
  grp.clearAccelerations();
  EXPECT_FALSE(grp.isValid(FieldTypes::ACCELERATION));
  grp.clearEfforts();
  EXPECT_FALSE(grp.isValid(FieldTypes::EFFORT));
  EXPECT_EQ(12, grp.byteLength());   // expected length: reqd fields

  // test copy and == operators
  EXPECT_NE(grp, grp2);              // unequal at first
  EXPECT_NO_THROW({grp2 = grp;});    // do assignment
  EXPECT_EQ(grp, grp2);              // equal after assignment
}

TEST(DynamicJointPt, init)
{
  using namespace industrial::dynamic_joint_pt;

  DynamicJointPt pt;
  DynamicJointPt emptyPt;
  pt.init();
  EXPECT_EQ(emptyPt, pt);  // constructor should also empty-initialize
  EXPECT_EQ(0, pt.getSequence());
  EXPECT_EQ(0, pt.getNumGroups());
  EXPECT_EQ(8, pt.byteLength());
  EXPECT_FALSE(pt.hasGroupID(2));
}

TEST(DynamicJointPt, assignment)
{
  using namespace industrial::dynamic_joint_pt;

  DynamicJointPt pt;
  DynamicJointPtGrp grp, grp2, emptyGrp;
  shared_real tmpVal, refVal = 3.14159;

  grp.init(2);
  grp.setGroupID(3);
  grp.setTime(refVal);

  pt.setSequence(42);
  EXPECT_EQ(42, pt.getSequence());              // check set/get sequence
  EXPECT_TRUE(pt.addGroup(grp));                // allow addGroup (new groupID)
  EXPECT_TRUE(pt.hasGroupID(3));                // check hasGroupID
  EXPECT_EQ(1, pt.getNumGroups());              // check numGroups
  EXPECT_EQ(grp, pt.getGroup(0));               // getGroup returns same data
  EXPECT_TRUE(pt.getGroup(0).getTime(tmpVal));  // allow read of group-data
  EXPECT_EQ(refVal, tmpVal);                    // check read data

  EXPECT_TRUE(pt.getGroup(0).setTime(refVal+1));  // allow write of group-data
  EXPECT_TRUE(pt.getGroup(0).getTime(tmpVal));    // allow read of modified data
  EXPECT_EQ(refVal+1, tmpVal);                    // check read data

  EXPECT_FALSE(pt.addGroup(grp));                // prevent add duplicate ID
  EXPECT_TRUE(grp.setGroupID(1));
  EXPECT_TRUE(pt.addGroup(grp));                 // allow add new ID
  EXPECT_NE(pt.getGroup(0), pt.getGroup(1));     // expect diff in time data

  EXPECT_ANY_THROW(pt.getGroup(-1));             // get unknown ID => create new empty group
  EXPECT_EQ(2, pt.getNumGroups());
}

TEST(DynamicJointPt, Comms)
{
  using namespace industrial::dynamic_joint_pt;
  using namespace industrial::dynamic_joint_pt_message;

  DynamicJointPt ptSend, ptRecv;
  DynamicJointPtMessage msgSend, msgRecv;

  ptSend.setSequence(1);
  DynamicJointPtGrp grp1;
  grp1.init(3);
  grp1.setGroupID(1);
  grp1.setTime(1.0);
  grp1.setPositions(std::vector<shared_real>(3, 1.25));
  ptSend.addGroup(grp1);
  DynamicJointPtGrp grp3;
  grp3.init(1);
  grp3.setGroupID(3);
  grp3.setTime(1.1);
  grp3.setPositions(std::vector<shared_real>(1, 12.5));
  ptSend.addGroup(grp3);
  msgSend.init(ptSend);

  messagePassing(msgSend, msgRecv);

  ptRecv = msgRecv.point_;
  ASSERT_EQ(ptSend, ptRecv);

}

TEST(DynamicJointState, grpInit)
{
  using namespace industrial::dynamic_joint_state;

  DynamicJointStateGrp grp;
  DynamicJointStateGrp emptyGrp;
  shared_real tmpVal;
  std::vector<shared_real> tmpArray;

  grp.init(0);
  EXPECT_TRUE(grp==emptyGrp);  // constructor should also empty-initialize
  EXPECT_EQ(0, grp.getNumJoints());
  EXPECT_EQ(0, grp.getGroupID());
  EXPECT_FALSE(grp.getActualPositions(tmpArray));
  EXPECT_FALSE(grp.getDesiredPositions(tmpArray));
  EXPECT_FALSE(grp.getPositionErrors(tmpArray));
  EXPECT_FALSE(grp.getActualVelocities(tmpArray));
  EXPECT_FALSE(grp.getDesiredVelocities(tmpArray));
  EXPECT_FALSE(grp.getVelocityErrors(tmpArray));
  EXPECT_FALSE(grp.getActualAccelerations(tmpArray));
  EXPECT_FALSE(grp.getDesiredAccelerations(tmpArray));
  EXPECT_FALSE(grp.getAccelerationErrors(tmpArray));
  EXPECT_FALSE(grp.getActualEfforts(tmpArray));
  EXPECT_FALSE(grp.getDesiredEfforts(tmpArray));
  EXPECT_FALSE(grp.getEffortErrors(tmpArray));
  EXPECT_EQ(12, grp.byteLength());

  grp.init(6,3);
  EXPECT_FALSE(grp==emptyGrp);
  EXPECT_EQ(6, grp.getNumJoints());
  EXPECT_EQ(3, grp.getGroupID());
  EXPECT_EQ(12, grp.byteLength());
}

TEST(DynamicJointState, grpAssignment)
{
  using namespace industrial::dynamic_joint_state;

  const int num_joints = 3;
  DynamicJointStateGrp grp, grp2;
  shared_real tmpVal, refVal;
  std::vector<shared_real> tmpArray, refArray, shortArray;

  refVal = 3.14159;
  for (int i=0; i<num_joints; ++i)
    refArray.push_back(i);
  shortArray = refArray; shortArray.pop_back();

  // test basic assignment expectations
  grp.init(num_joints);
  EXPECT_TRUE(grp.setGroupID(4));
  EXPECT_EQ(4, grp.getGroupID());
  EXPECT_EQ(grp.getNumJoints(), num_joints);        // init should set num_joints
  EXPECT_FALSE(grp.setActualPositions(shortArray)); // prevent invalid value
  EXPECT_TRUE(grp.setActualPositions(refArray));    // allow set valid value
  EXPECT_TRUE(grp.getActualPositions(tmpArray));    // allow read valid value
  EXPECT_EQ(refArray, tmpArray);                    // check write/read match
  EXPECT_FALSE(grp.setActualVelocities(shortArray));
  EXPECT_TRUE(grp.setActualVelocities(refArray));
  EXPECT_TRUE(grp.setActualVelocities(tmpArray));
  EXPECT_EQ(refArray, tmpArray);
  EXPECT_FALSE(grp.setActualAccelerations(shortArray));
  EXPECT_TRUE(grp.setActualAccelerations(refArray));
  EXPECT_TRUE(grp.setActualAccelerations(tmpArray));
  EXPECT_EQ(refArray, tmpArray);
  EXPECT_FALSE(grp.setActualEfforts(shortArray));
  EXPECT_TRUE(grp.setActualEfforts(refArray));
  EXPECT_TRUE(grp.setActualEfforts(tmpArray));
  EXPECT_EQ(refArray, tmpArray);
  EXPECT_FALSE(grp.setDesiredPositions(shortArray));
  EXPECT_TRUE(grp.setDesiredPositions(refArray));
  EXPECT_TRUE(grp.getDesiredPositions(tmpArray));
  EXPECT_EQ(refArray, tmpArray);
  EXPECT_FALSE(grp.setDesiredVelocities(shortArray));
  EXPECT_TRUE(grp.setDesiredVelocities(refArray));
  EXPECT_TRUE(grp.setDesiredVelocities(tmpArray));
  EXPECT_EQ(refArray, tmpArray);
  EXPECT_FALSE(grp.setDesiredAccelerations(shortArray));
  EXPECT_TRUE(grp.setDesiredAccelerations(refArray));
  EXPECT_TRUE(grp.setDesiredAccelerations(tmpArray));
  EXPECT_EQ(refArray, tmpArray);
  EXPECT_FALSE(grp.setDesiredEfforts(shortArray));
  EXPECT_TRUE(grp.setDesiredEfforts(refArray));
  EXPECT_TRUE(grp.setDesiredEfforts(tmpArray));
  EXPECT_EQ(refArray, tmpArray);
  EXPECT_FALSE(grp.setPositionErrors(shortArray));
  EXPECT_TRUE(grp.setPositionErrors(refArray));
  EXPECT_TRUE(grp.getPositionErrors(tmpArray));
  EXPECT_EQ(refArray, tmpArray);
  EXPECT_FALSE(grp.setVelocityErrors(shortArray));
  EXPECT_TRUE(grp.setVelocityErrors(refArray));
  EXPECT_TRUE(grp.setVelocityErrors(tmpArray));
  EXPECT_EQ(refArray, tmpArray);
  EXPECT_FALSE(grp.setAccelerationErrors(shortArray));
  EXPECT_TRUE(grp.setAccelerationErrors(refArray));
  EXPECT_TRUE(grp.setAccelerationErrors(tmpArray));
  EXPECT_EQ(refArray, tmpArray);
  EXPECT_FALSE(grp.setEffortErrors(shortArray));
  EXPECT_TRUE(grp.setEffortErrors(refArray));
  EXPECT_TRUE(grp.setEffortErrors(tmpArray));
  EXPECT_EQ(refArray, tmpArray);
  #ifndef FLOAT64
  EXPECT_EQ(12 + 48*num_joints, grp.byteLength());  // expected length (full msg)
  #else
  EXPECT_EQ(12 + 96*num_joints, grp.byteLength());  // expected length (full msg)
  #endif

  // test the ability to clear data arrays
  grp.clearActualPositions();                       // clear data array
  EXPECT_FALSE(grp.isValid(FieldTypes::POSITION));  // should be marked invalid
  grp.clearActualVelocities();
  EXPECT_FALSE(grp.isValid(FieldTypes::VELOCITY));
  grp.clearActualAccelerations();
  EXPECT_FALSE(grp.isValid(FieldTypes::ACCELERATION));
  grp.clearActualEfforts();
  EXPECT_FALSE(grp.isValid(FieldTypes::EFFORT));
  grp.clearDesiredPositions();
  EXPECT_FALSE(grp.isValid(FieldTypes::POSITION_DESIRED));
  grp.clearDesiredVelocities();
  EXPECT_FALSE(grp.isValid(FieldTypes::VELOCITY_DESIRED));
  grp.clearDesiredAccelerations();
  EXPECT_FALSE(grp.isValid(FieldTypes::ACCELERATION_DESIRED));
  grp.clearDesiredEfforts();
  EXPECT_FALSE(grp.isValid(FieldTypes::EFFORT_DESIRED));
  grp.clearPositionErrors();
  EXPECT_FALSE(grp.isValid(FieldTypes::POSITION_ERROR));
  grp.clearVelocityErrors();
  EXPECT_FALSE(grp.isValid(FieldTypes::VELOCITY_ERROR));
  grp.clearAccelerationErrors();
  EXPECT_FALSE(grp.isValid(FieldTypes::ACCELERATION_ERROR));
  grp.clearEffortErrors();
  EXPECT_FALSE(grp.isValid(FieldTypes::EFFORT_ERROR));
  EXPECT_EQ(12, grp.byteLength());   // expected length: reqd fields

  // test copy and == operators
  EXPECT_NE(grp, grp2);              // unequal at first
  EXPECT_NO_THROW({grp2 = grp;});    // do assignment
  EXPECT_EQ(grp, grp2);              // equal after assignment
}

TEST(DynamicJointState, init)
{
  using namespace industrial::dynamic_joint_state;

  DynamicJointState state;
  DynamicJointState emptyState;
  state.init();
  EXPECT_EQ(emptyState, state);  // constructor should also empty-initialize
  EXPECT_EQ(0, state.getSequence());
  EXPECT_EQ(0, state.getNumGroups());
  EXPECT_EQ(8, state.byteLength());
  EXPECT_FALSE(state.hasGroupID(2));
  EXPECT_ANY_THROW(state.getGroup(2));
}


TEST(DynamicJointState, assignment)
{
  using namespace industrial::dynamic_joint_state;

  DynamicJointState state;
  DynamicJointStateGrp grp, grp2, emptyGrp;
  std::vector<shared_real> tmpArray, refArray;

  for (int i=0; i<2; ++i)
    refArray.push_back(i);

  grp.init(2, 3);
  grp.setActualPositions(refArray);

  state.setSequence(42);
  EXPECT_EQ(42, state.getSequence());              // check set/get sequence
  EXPECT_TRUE(state.addGroup(grp));                // allow addGroup (new groupID)
  EXPECT_TRUE(state.hasGroupID(3));                // check hasGroupID
  EXPECT_EQ(1, state.getNumGroups());              // check numGroups
  EXPECT_EQ(grp, state.getGroup(0));               // getGroup returns same data
  EXPECT_TRUE(state.getGroup(0).getActualPositions(tmpArray));  // read data
  EXPECT_EQ(refArray, tmpArray);                   // check read data

  refArray[0] += 1;
  EXPECT_TRUE(state.getGroup(0).setActualPositions(refArray));  // allow write
  EXPECT_TRUE(state.getGroup(0).getActualPositions(tmpArray));  // allow read
  EXPECT_EQ(refArray, tmpArray);                                // check data

  EXPECT_FALSE(state.addGroup(grp));                // prevent add duplicate ID
  grp.setGroupID(1);
  EXPECT_TRUE(state.addGroup(grp));                 // allow add new ID
  EXPECT_NE(state.getGroup(0), state.getGroup(1));  // diff in position data

  EXPECT_EQ(2, state.getNumGroups());
}

TEST(DynamicJointState, Comms)
{
  using namespace industrial::dynamic_joint_state;
  using namespace industrial::dynamic_joint_state_message;

  DynamicJointState stateSend, stateRecv;
  DynamicJointStateMessage msgSend, msgRecv;

  stateSend.setSequence(1);
  DynamicJointStateGrp grp1(3, 1);
  grp1.setActualPositions(std::vector<shared_real>(3, 1.25));
  stateSend.addGroup(grp1);
  DynamicJointStateGrp grp3(1, 3);
  grp3.setActualPositions(std::vector<shared_real>(1, 12.5));
  stateSend.addGroup(grp3);
  msgSend.init(stateSend);

  messagePassing(msgSend, msgRecv);

  stateRecv = msgRecv.state_;
  ASSERT_EQ(stateSend, stateRecv);

}


TEST(DynamicGroupStatus, init)
{
  using namespace industrial::dynamic_group_status;

  DynamicGroupStatus status;
  DynamicGroupStatus emptyStatus;
  status.init();
  EXPECT_EQ(emptyStatus, status);  // constructor should also empty-initialize
  EXPECT_EQ(0, status.getNumGroups());
  EXPECT_EQ(4, status.byteLength());
  EXPECT_FALSE(status.hasGroupID(2));
}


TEST(DynamicGroupStatus, assignment)
{
  using namespace industrial::dynamic_group_status;
  using namespace industrial::robot_status;

  DynamicGroupStatus state;
  DynamicGroupStatusGrp grp, grp2, emptyGrp;
  shared_int tmpVal;

  grp.init(TriStates::TS_ENABLED, TriStates::TS_FALSE, 99, TriStates::TS_TRUE,
           TriStates::TS_TRUE, RobotModes::MANUAL, TriStates::TS_DISABLED);
  grp.setGroupID(3);

  EXPECT_TRUE(state.addGroup(grp));                // allow addGroup (new groupID)
  EXPECT_TRUE(state.hasGroupID(3));                // check hasGroup
  EXPECT_EQ(1, state.getNumGroups());              // check numGroups
  EXPECT_EQ(grp, state.getGroup(0));               // getGroup returns same data
  EXPECT_EQ(99, state.getGroup(0).getErrorCode()); // check read data

  state.getGroup(0).setErrorCode(100);
  EXPECT_EQ(100, state.getGroup(0).getErrorCode()); // check write/read

  EXPECT_FALSE(state.addGroup(grp));                // prevent add duplicate ID
  grp.setGroupID(1);
  EXPECT_TRUE(state.addGroup(grp));                 // allow add new ID
  EXPECT_NE(state.getGroup(0), state.getGroup(1));  // expect diff in errorCode

  EXPECT_ANY_THROW(state.getGroup(-1));             // get unknown idx => throw exception
  EXPECT_EQ(2, state.getNumGroups());
}

TEST(DynamicGroupStatus, Comms)
{
  using namespace industrial::dynamic_group_status_message;
  using namespace industrial::dynamic_group_status;

  DynamicGroupStatusGrp grp1, grp3;
  DynamicGroupStatus statusSend, statusRecv;
  DynamicGroupStatusMessage msgSend, msgRecv;

  grp1.init(TriStates::TS_ENABLED, TriStates::TS_FALSE, 99, TriStates::TS_TRUE,
            TriStates::TS_TRUE, RobotModes::MANUAL, TriStates::TS_DISABLED);
  grp1.setGroupID(1);
  statusSend.addGroup(grp1);

  grp3.init(TriStates::TS_ENABLED, TriStates::TS_TRUE, 66, TriStates::TS_TRUE,
            TriStates::TS_FALSE, RobotModes::AUTO, TriStates::TS_ENABLED);
  grp3.setGroupID(3);
  statusSend.addGroup(grp3);
  msgSend.init(statusSend);

  messagePassing(msgSend, msgRecv);

  statusRecv = msgRecv.status_;
  ASSERT_EQ(statusSend, statusRecv);

}
