// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double LOOP_PERIOD = 0.02; // 20ms
  public static final double LOOP_FREQUENCY = 1.0 / LOOP_PERIOD; // 50Hz
  public static final double NOMINAL_VOLTAGE = 12; // V
  public static final double g = 9.79267; // m/s^2 in Pearland

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double UPDATE_FREQ_SEC = 0.02;

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

  public static final class VisualizerConstants {
        public static final Translation3d MODEL0_ZERO = new Translation3d(-0.134550, -0.143323, 0);
        public static final Translation3d Z1_ZERO = new Translation3d(-0.1235075, -0.041317, 0.519888);
        public static final Translation3d MODEL2_ZERO = new Translation3d(0.024588, 0, 0);
        public static final Translation3d MODEL3_ZERO = new Translation3d(0.205374, 0, 0.260350);
        public static final Translation3d Z4_ZERO = new Translation3d(0.302910, 0, 0.646415);

        public static final Translation3d MODEL1_OFFSET = Z1_ZERO.minus(MODEL0_ZERO);
        public static final Translation3d MODEL4_OFFSET = Z4_ZERO.minus(MODEL3_ZERO);

        public static final double TURRET_STARTING_ANGLE = Math.PI / 2;

        public static final double HOOD_STARTING_ANGLE = Units.degreesToRadians(61.549451);
        public static final double HOOD_MIN_ANGLE = Units.degreesToRadians(24.652849);
        public static final double HOOD_MAX_ANGLE = Units.degreesToRadians(69.652849);

        public static final double INTAKE_STARTING_ANGLE = Math.PI / 2; // radians
        public static final double GRAVITY_RAMP_MAX_OFFSET_DEGS = 39; // degrees

        public static final double CLIMBER_MAX_DISPLACEMENT = Units.inchesToMeters(5.875);
    }
}
