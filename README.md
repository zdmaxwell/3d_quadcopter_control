## Project: Control of a 3D Quadrotor

### Scenario 1 (Hover):

![Demo](videos/scenario%201%20gif.gif)

For the initial scenario, I updated the mass in `QuadControlParams.txt` so that the quad

### Scenario 2 (Body Rate and Roll/Pitch Control):

![Demo](videos/scenario%202%20gif.gif)

For the `GenerateMotorCommands()`, I calculated the thrust components in the x, y, and z direction using the provided commanded collective thrust and the moments about the x, y, and z axis;
Using the relationship between Moments and Force ( Force = Moment / L_arm ), I determined the forces that corresponded to the moments in the x, y, and z direction. The x and y moments factored in the arm length
of the quadrotor, where the arm length was equal to l, since the x and y coordinate frame tied to the quadrotor is rotated at 45 degrees with respect to the quadrotor's arms.

```
float l = L / sqrt(2.f);

float Fx = momentCmd.x / l;
float Fy = momentCmd.y / l;
float Fz = momentCmd.z / kappa;
```

### Scenario 3 (Position/velocity and yaw angle control):

![Demo](videos/scenario%204%20gif.gif)

### Scenario 4 (Non-Idealities):

![Demo](videos/scenario%203%20gif.gif)

### Scenario 5 (Tracking trajectories):

![Demo](videos/scenario%205%20gif.gif)
