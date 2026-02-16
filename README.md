# 2026 Pearadox CompBot

## Shoot On The Move (SOTM)

This year, our software team has been experimenting with "Shoot On the Move." The premise is simple: ensure that our swerve robot can accurately and precisely launch fuel into the hub simultaneously while it drives around on the field. This is especially useful with a turretted shooter since our robot can be intaking and launching fuel at the same time which may be useful during intense matches.

Throughout the course of the season, there have been some key lessons, ideas, and takeaways that we've learned that we'd like to share:

### Week 1-2: Physics-Based Model

- Decided to initially implement a physics-based projectile model with a constant hood angle.
- Used Newton's Method (calculus technique) and kinematics to estimate the Time-of-Flight (airtime) for the fuel based on the robot's translational velocity and distance to the target from Limelight odometry updates
- Derived required field-relative shooter launch velocities (vx, vy, vz) that compensate for translational robot motion, then computed total shooter wheel speed and a field-relative turret angle to “lead” the target.
- Output a "ShotSolution" containing time-of-flight, scaled shooter speed, and turret angle, while logging intermediate values for debugging and visualization.


### Week 3-4: Real-World Corrections
- Observed shooter underperformance due to energy transfer inefficiencies.
- Introduced empirical scaling factor(s) (multiplying by a constant) to correct ShotSolution outputs in the real-world.
- Tested bang-bang velocity control for faster spin-up (full power if shooter spinning too slow, zero power if too fast).
- Identified oscillation issues near setpoint with the bang-bang approach.

### Week 5-6:
TODO  (interpolation methods vs physics calculations maybe?)

### Lessons Learned:
- Physics models provide strong starting points but often require empirical correction.
- Energy transfer efficiency significantly affects real-world projectile accuracy.
- Bang-bang control can reduce spin-up time but increases steady-state oscillation.
- TODO