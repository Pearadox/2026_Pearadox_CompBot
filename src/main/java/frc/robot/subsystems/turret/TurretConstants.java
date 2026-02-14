package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class TurretConstants {
        public static final TalonFXConfiguration getTurretConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 50;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 50;

            config.MotionMagic.MotionMagicCruiseVelocity = 20;
            config.MotionMagic.MotionMagicAcceleration = 75;

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            config.Slot0.kG = 0.0;
            config.Slot0.kS = 0.0;
            config.Slot0.kV = 0.0;
            config.Slot0.kA = 0.0;
            config.Slot0.kP = 0.67;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.05;

            return config;
        }

        public static final int TURRET_ID = 20;
        public static final double TURRET_GEAR_RATIO = 95. / 10.;
        public static final double TURRET_P_COEFFICIENT = 2 * Math.PI / TURRET_GEAR_RATIO;

        public static final double TURRET_STARTING_ANGLE = Units.degreesToRadians(0);
        public static final double TURRET_MIN_ANGLE = Units.degreesToRadians(-270);
        public static final double TURRET_MAX_ANGLE = Units.degreesToRadians(270);

        public static final double TURRET_MASS = Units.lbsToKilograms(16);
        public static final double TURRET_CG_RADIUS = Units.inchesToMeters(3.75);

        // mass ≈ 16 lb, Lzz ≈ 494 in^2 lb
        // center of mass of turret ≈ 3.75 in from its axis of rotation
        // I = I_cm + md^2 = 494 + 16(3.75)^2 = 719 in^2 lb ≈ 0.21 kg m^2
        public static final double TURRET_MOI = 0.21;

        public static final DCMotor TURRET_MOTOR = DCMotor.getKrakenX60(1);

        // feedforward term: adds a voltage to the turret as the chassis rotates
        public static final double K_OMEGA = 0.1; // volts per radian per second

        public static final double SAFETY_LIMIT = Units.degreesToRadians(5);
        public static final double TURRET_SAFE_MIN = TURRET_MIN_ANGLE + SAFETY_LIMIT;
        public static final double TURRET_SAFE_MAX = TURRET_MAX_ANGLE - SAFETY_LIMIT;

        // only apply feedforward if the turret is within 45 degrees of its setpoint
        public static final double FF_ERROR_THRESHOLD = Units.degreesToRadians(45);
        
        // only apply feedforward if the drivetrain is rotating at a reasonable speed
        // note: this may not be necessary
        public static final double FF_CHASSIS_ROT_VELOCITY_LIMIT = 1.5 * Math.PI; // rad/s
    }   
