## Project: Control of a 3D Quadrotor

Quadrotor Coordinate System
![Coordinate system top view](diagrams/drone_coordinate_system1.png)
![Coordinate system side view](diagrams/drone_coordinate_system2.png)

### Scenario 1 (Hover):

![Scenario 1](videos/scenario%201%20gif.gif)

For the initial scenario, I updated the mass in `QuadControlParams.txt` so that the quad

### Scenario 2 (Body Rate and Roll/Pitch Control):

![Scenario 2](videos/scenario%202%20gif.gif)

For the `GenerateMotorCommands()` method, I calculated the thrust components in the x, y, and z direction using the provided commanded collective thrust and the moments about the x, y, and z axis (roll, pitch, and yaw);
Using the relationship between Moments and Force ( Force = Moment / L_arm ), I determined the forces (thrust) that corresponded to the moments in the x, y, and z direction. The x and y moments factored in the arm length
of the quadrotor, where the arm length was equal to L / sqrt(2), since the x and y coordinate frame tied to the quadrotor is rotated at 45 degrees with respect to the quadrotor's arms. The moment in the z direction
is dependent on the drag to thrust ratio, kappa, since the propellers inherently induce a moment about the z axis while spinning.

```
float l = L / sqrt(2.f);

float Fx = momentCmd.x / l;
float Fy = momentCmd.y / l;
float Fz = momentCmd.z / kappa;
```

To apply the proper thrusts to each of the four propellers, I assigned a combination of the commanded collective thrust and the force component in each direction (x, y, z). I assigned the signs of the thrust components
by determining how each thrust component contributes to the roll, pitch, and yaw moments based on the coordinate system. For example, to induce a positive roll moment (x direction), the thrust component for the front left
and rear left propellers need to be positive to cause the quad to rotate about the x axis. Similarly, for the z direction thrust, the front right and rear left propellers spin out of the screen, so they induce a positive
reactive moment in the z direction, hence the positive z direction thrust component.

```
cmd.desiredThrustsN[0] = (collThrustCmd + Fx + Fy - Fz) / 4.f; // front left
cmd.desiredThrustsN[1] = (collThrustCmd - Fx + Fy + Fz) / 4.f; // front right
cmd.desiredThrustsN[2] = (collThrustCmd + Fx - Fy + Fz) / 4.f; // rear left
cmd.desiredThrustsN[3] = (collThrustCmd - Fx - Fy - Fz) / 4.f; // rear right
```

To develop the `BodyRateControl()` controller method, I used a P controller since this controller is a 1st order system as it takes in rotation rates in the body frame (p, q, r) which are the first
derivative of the rotational position, and outputs commanded moments that are directly proportional to the body rotation rates. By determining the `bodyRateError` `V3F` object using the provided `pqrCmd` and `pqr` values,
packaging the moments of inertia into a `V3F` data type, and factoring in the `kpPQR` gain, I calculated the commanded moment and tuned the gain values for `kpPQR` until the quad could reach the set altitude while controlling it's
body frame rotations with minimal overshoot.

```
V3F momentCmd;
V3F momentOfInertia(Ixx, Iyy, Izz);
V3F bodyRateError = pqrCmd - pqr;
momentCmd = momentOfInertia * kpPQR * bodyRateError;
```

Lastly, to develop the `RollPitchControl()` controller, I implemented another P controller as it was also a 1st order system to control the commanded angular velocities `p_c` and `q_c` in the body frame.
To obtain the actual body frame roll and pitch angles, I referenced the provided rotation matrix, `R` which maps the body frame orientations to the inertial frame orientations. Since the third column of the rotation matrix represents the
mapping of the body frame orientations's z axis to the correcsponding x and y world frame axes, the actual thrust direction in the world frame is equal to R13 and R23.

```
float b_x_act = R(0, 2); // R13
float b_y_act = R(1, 2); // R23
```

Since the commanded roll and pitch angles were not directly accessible, I solved for them using the relationship between the collective thrust, linear acceleration, and the collective thrust direction (rads) for the x and y axis.

```
x_dot_dot = Cc * b_x_cmd
y_dot_dot = Cc * b_y_cmd
```

After normalizing the collectiveThrustCmd by the quad's mass, I rearranged the above equations to solve for the commanded collective thrust directions in the x and y axis. And to ensure the angles maintained
safe values, I constrained them so the quad couldn't rotate too far to attempt to achieve a greater acceleration.

```
float z_accel_cmd_bf = -collThrustCmd / mass;
float b_x_cmd = CONSTRAIN((accelCmd.x / z_accel_cmd_bf), -maxTiltAngle, maxTiltAngle);
float b_y_cmd = CONSTRAIN((accelCmd.y / z_accel_cmd_bf), -maxTiltAngle, maxTiltAngle);
```

Solving for the commanded rates of change of the direction of thrust along the x and y axis, I used a P controller where the error terms were equal to the difference between the commanded and actual collective thrust directions in the x and y axis, and
with a single gain term, `kpBank`.

```
float b_dot_x_cmd = kpBank * (b_x_cmd - b_x_act);
float b_dot_y_cmd = kpBank * (b_y_cmd - b_y_act);
```

To output the commanded angular velocities for the quadrotor's body frame, `p_c` and `q_c`, I use the derived equation that relates the rate of change of the attitudes in the rotation matrix `R` (`b_dot_x` and `b_dot_y`) to the commanded body frame angular velocities, `p_c` and `q_c`.
Solving for `p_c` and `q_c` takes the form below while ensuring the angular velocity in the z direction is zero:

```
pqrCmd.x = (R(1, 0) * b_dot_x_cmd - R(0, 0) * b_dot_y_cmd) / R(2, 2);
pqrCmd.y = (R(1, 1) * b_dot_x_cmd - R(0, 1) * b_dot_y_cmd) / R(2, 2);
pqrCmd.z = 0.f;
```

Then I tuned `kpBank` to the point where the quadrotor maintained a level hover in scenario 2.

### Scenario 3 (Position/velocity and yaw angle control):

![Scenario 3](videos/scenario%204%20gif.gif)

Since the `LateralPositionControl()` controller is a 2nd order system, I implemented a PD controller with feedforward control to output the commanded acceleration in the x and y direction.
I also constrained the commanded velocity values to the quadrotor's actual range in the x and y direction and calculated the errors for the `P` and `D` terms below. Finally, I constrained the commanded
x and y accelerations to the quadrotor's actual ranges.

```
V3F posError = posCmd - pos; // P term error
V3F velError = velCmd - vel; // D term error

velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);

accelCmd.x += kpPosXY * posError.x + kpVelXY * velError.x; // PD controller for x direction
accelCmd.y += kpPosXY * posError.y + kpVelXY * velError.y; // PD controller for y direction

accelCmd.x += CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
accelCmd.y += CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
```

To implement the `AltitudeControl()` controller, I used a PID controller with feedforward control for both the commanded velocity and acceleration to command the z direction thrust. The benefit of the added
feedforward terms is that the controller can more quickly meet the desired set points for commanded acceleration than a feedback control system which inherently needs to wait for the error terms to accumulate to produce
a control output.

I calculated the PID error terms as follows:

```
float z_error = posZCmd - posZ;
float z_dot_error = velZCmd - velZ;
integratedAltitudeError += z_error * dt;
```

The PID controller with feedforward control takes the following form as described above:

```
float u_bar_1 = kpPosZ * z_error + KiPosZ * integratedAltitudeError + kpVelZ * z_dot_error + velZCmd + accelZCmd;
```

Finally, to output the commanded collective thrust, I used the relationship between the vertical acceleration (z_dot_dot), acceleration due to gravity, and collective thrust.
With the NED frame, the z acceleration corresponding to the collective thrust is equal to the below equation, where b_z is equal to the term in the rotation matrix `R` that determines z direction thrust in the inertial frame.
For instance, if the quadrotor is level, then b_z will be 1 since the body z axis aligns with the inertial z axis. If the quadrotor is tilted about the z axis, then there is some component of roll, pitch, or both. Since R33 = cos(theta) \* cos(phi),
the roll and pitch angles correspond directly to the value of R33 which will scale the thrust based on that total tilt angle.
The sign for the `thrust` equation ensures that the thrust and acceleration matches the NED frame convention where acceleration due to gravity is assumed to be positive (downward vector), and the acceleration control effort `u_bar_1` is assumed to be negative (upward vector).
For instance, as the quadrotor is ascending to a desired altitude, the `AltitudeControl()` controller will be providing a larger and larger control effort `u_bar_1` to counteract gravity. Eventually, `u_bar_1` will be greater than the acceleration due to the gravity which translate to
a negative thrust vector (pointing upwards in the NED frame). Lastly, I constrained the acceleration due to thrust by solving for the maxAccelZ parameter using `maxAscentRate / dt` which results in an acceleration [m / s^2].

```
float b_z = R(2, 2); R33
thrust = CONSTRAIN(((CONST_GRAVITY - u_bar_1) / b_z), -maxAccelZ, maxAccelZ) * mass;
```

Accounting for `YawControl()`, I developed a P controller since the controller was a 1st order system which needed to output a `yawRateCmd`.
This P controller, however, required some additional constraining measures to ensure that larger, unnecessary control efforts were not outputted.
To start, I constrained the `yawCmd` and `yaw` values to maintain a range of `[0, 2*PI]` since that would account for any possible yaw orientation that the quadrotor would achieve, and then I solved for the `yawError` term.

```
yawCmd = fmodf(yawCmd, 2 * F_PI);
yaw = fmodf(yaw, 2 * F_PI);
float yawError = yawCmd - yaw;
```

As an additional precautionary measure, I further constrained the yawError term to essentially find the shortest path to the commanded yaw value. To provide an example of an extreme case, if the commanded yaw value was at 6 rads and
the actual yaw value was at 0.5 rads, the `yawError` term would equal 5.5 rads. Without any constraining methods, the control effort would be much larger than necessary because there is a shorter path from 0.5 rads to 6 rads than traversing 5.5 rads.
This method checks to see if the `yawError` term is greater than or less than `+ or - PI` respectively. If that case evaluates to true, then a shorter path is available, and in the case of the example, the quadrotor would be able to instead provide a commanded
yaw of -0.78 rads.

```
if (yawError > F_PI)
{
yawError -= 2 * F_PI;
}
if (yawError < -F_PI)
{
yawError += 2 * F_PI;
}
```

Lastly, to output the `yawRatecmd`, I used a P controller of the following form:

```
yawRateCmd = kpYaw * yawError;
```

Tuning...

### Scenario 4 (Non-Idealities):

![Scenario 4](videos/scenario%203%20gif.gif)

### Scenario 5 (Tracking trajectories):

![Scenario 5](videos/scenario%205%20gif.gif)
