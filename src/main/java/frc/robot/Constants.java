// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class AlignConstants {
        public static final double ALIGN_STRAFE_KP = 0.02;
        public static final double ALIGN_STRAFE_KI = 0.001;
        public static final double ALIGN_FORWARD_KP = 0.06; // -0.06
        public static final double ALIGN_KS = 0.009;

        // rotational PID
        public static final double ROT_kP = 24.0;
        public static final double ROT_kI = 0.0;
        public static final double ROT_kD = 0.0;
        public static final double MAX_ROT_VELOCITY = Math.PI * 2; // rad/s
        public static final double MAX_ROT_ACCELERATION = Math.PI * 4; // rad/s^2
  }
}
