#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"
#include <iostream>

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;

#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();

  // Load parameters (default to 0)
  kpPosXY = config->Get(_config + ".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);

  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  // TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to
  //   individual motor thrust commands
  // INPUTS:
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS:
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  // Summary: Since I know the commanded moments in each direction and the commanded collective thrust, I use the relationship between Moment and Force
  // to solve for each thrust force component in the x, y, and z directions. Then, I use the coordinate system to determine what the sign of each thrust
  // component would be for each propeller based on how each thrust component contributes to the roll, pitch, and yaw moments. For example, to induce a positive
  // roll moment (x-direction), the thrust component in the x-direction (Fx) for the front left and rear left propellers will need to be positive.

  float l = L / sqrt(2.f); // perpendicular distance for the rotor arm length L rotated at 45 degrees in coordinate system

  // Calculate the force components in each direction with F = M / L [N]. Here L = l since the drone is rotated at 45 degrees in the coordinate system.
  // The one caveat is that in the z-direction, the yaw moment Mz, is not dependent on the arm length L, but is dependent on the term kappa (drag/thrust ratio)
  // since the spinning rotors inherently induce a moment in the z-direction and the sign for that moment depends on the spin direction of propellers
  float Fx = momentCmd.x / l;
  float Fy = momentCmd.y / l;
  float Fz = momentCmd.z / kappa;

  // Spread the forces out equally among all four propellers
  // Each propeller receives a force contribution from the collective thrust and x, y, z moments
  cmd.desiredThrustsN[0] = (collThrustCmd + Fx + Fy - Fz) / 4.f; // front left
  cmd.desiredThrustsN[1] = (collThrustCmd - Fx + Fy + Fz) / 4.f; // front right
  cmd.desiredThrustsN[2] = (collThrustCmd + Fx - Fy + Fz) / 4.f; // rear left
  cmd.desiredThrustsN[3] = (collThrustCmd - Fx - Fy - Fz) / 4.f; // rear right

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS:
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS:
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  // Summary: This controller is taking in the commanded body rotation rates and the current body rotation rates. Since I have the measured moment of inertia
  // values, I can build a P controller to control the body rotation rates (roll, pitch, yaw) by feeding the error between the desired and actual body rotation rates
  // to the controller to calculate the commanded moment following the equation M = I x a. This controller is a 1st order system since it receives rotation rates
  // (1st derivative of position) and sends commanded moments in x, y, z directions which are directly proportional to the body rotation rates.
  V3F momentCmd;

  // create moments of inertia V3F object
  V3F momentOfInertia(Ixx, Iyy, Izz);

  // calculate the error between the desired and actual body rates (p, q, r) which are body-frame angular velocities [rad/s]
  V3F bodyRateError = pqrCmd - pqr;

  // implements a P controller to scale the error by a factor of kpPQR in each direction; larger errors result in larger control responses
  // update x, y, z components of momentCmd by calculating the commanded moment in each direction scaled by P controller [N*m]
  momentCmd = momentOfInertia * kpPQR * bodyRateError;

  return momentCmd;
}

// returns a desired roll and pitch rate
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS:
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS:
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  // Summary: 1st order system P controller
  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB(); // rotation matrix from body frame to inertial frame

  // get R13 and R23 which correspond to the direction of the collective thrust in the intertial frame
  float b_x_act = R(0, 2); // R13
  float b_y_act = R(1, 2); // R23

  // convert collThrustCmd to vertical acceleration in the body frame (i.e. b_x and b_y are unitless params)
  // collThrustCmd is upward, so need to represent the corresponding acceleration to match the convention (upward is negative in NED)
  float z_accel_cmd_bf = -collThrustCmd / mass;

  // calculate the commanded values of b_x_cmd and b_y_cmd (use the relationship x_dot_dot = Cc *  b_x_cmd and y_dot_dot = Cc * b_y_cmd)
  // constrain the direction of thrust to safe angles
  float b_x_cmd = CONSTRAIN((accelCmd.x / z_accel_cmd_bf), -maxTiltAngle, maxTiltAngle);
  float b_y_cmd = CONSTRAIN((accelCmd.y / z_accel_cmd_bf), -maxTiltAngle, maxTiltAngle);

  // solve for rate of change of R13 and R23, b_dot_x and b_dot_y (desired rates of change of the collective thrust)
  float b_dot_x_cmd = kpBank * (b_x_cmd - b_x_act);
  float b_dot_y_cmd = kpBank * (b_y_cmd - b_y_act);

  // solve for commanded roll and pitch rates in the body frame, pc and qc
  pqrCmd.x = (R(1, 0) * b_dot_x_cmd - R(0, 0) * b_dot_y_cmd) / R(2, 2);
  pqrCmd.y = (R(1, 1) * b_dot_x_cmd - R(0, 1) * b_dot_y_cmd) / R(2, 2);
  pqrCmd.z = 0.f;

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical
  //   acceleration feed-forward command
  // INPUTS:
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS:
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  // Summary: 2nd order system. Using a PID controller with feedforward control. This controller returns a collective thrust that is required to achieve a
  // desired acceleration. Using the z_dot_dot linear acceleration to solve for the collective thrust command
  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  // R33: Element in rotation matrix responsible for translating thrust in body frame to global frame.It determines how much thrust is acting in the global z direction based on the tilt of the quad
  // i.e. 1) quadcopter level: R33 = 1 since the body z axis aligns with the global z axis, 2) quad tilted: R33 < 1 since body z axis is not aligned with the global z axis
  // R33 = cos(theta) * cos(phi) which accounts for the pitch and roll angle about the y and x axis, respectively.
  float b_z = R(2, 2);

  // calculate the position and velocity error terms for the P and D part of the controller
  float z_error = posZCmd - posZ;
  float z_dot_error = velZCmd - velZ;

  // sum up the position error over time to reduce steady state error and implement the I term of the controller
  integratedAltitudeError += z_error * dt;

  // PID controller to calculate the acceleration control effort with feedforward control for the commanded velocity and acceleration
  float u_bar_1 = kpPosZ * z_error + KiPosZ * integratedAltitudeError + kpVelZ * z_dot_error + velZCmd + accelZCmd;

  // calculate the commanded collective thrust based on the acceleration control effort in the z direction
  // constrain the mass normalized thrust (acceleration) to safe values even if the position, velocity or integral errors are high
  // use the constrained acceleration to calculate the commanded thrust by introducing the mass component
  float maxAccelZ = maxAscentRate / dt;
  thrust = CONSTRAIN(((CONST_GRAVITY - u_bar_1) / b_z), -maxAccelZ, maxAccelZ) * mass;

  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS:
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations.
  //     the Z component should be 0
  // HINTS:
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  // limit commanded velocities to the vehicle's actual range
  velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
  velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);

  // calculate the position and velocity error terms for x and y (z component is handled above)
  V3F posError = posCmd - pos;
  V3F velError = velCmd - vel;

  // implement a PD controller with feedforward term to calculate the commanded x and y acceleration control signals
  accelCmd.x += kpPosXY * posError.x + kpVelXY * velError.x;
  accelCmd.y += kpPosXY * posError.y + kpVelXY * velError.y;

  // limit commanded accelerations to the vehicle's actual range
  accelCmd.x += CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
  accelCmd.y += CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS:
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS:
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b].
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd = 0;

  yawCmd = fmodf(yawCmd, 2 * F_PI);
  yaw = fmodf(yaw, 2 * F_PI);

  // calculate the yawError
  float yawError = yawCmd - yaw;

  // check if there's a shorter path to the commanded yaw to negate large, unnecessary corrections
  if (yawError > F_PI)
  {
    yawError -= 2 * F_PI;
  }
  if (yawError < -F_PI)
  {
    yawError += 2 * F_PI;
  }

  // implement a P controller (1st order system) to output the commanded yaw rate control signal
  yawRateCmd = kpYaw * yawError;

  return yawRateCmd;
}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f * (maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust + thrustMargin) * 4.f, (maxMotorThrust - thrustMargin) * 4.f);

  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);

  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
