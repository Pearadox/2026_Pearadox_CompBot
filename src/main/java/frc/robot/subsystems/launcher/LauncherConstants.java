// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import edu.wpi.first.math.util.Units;
import lombok.Getter;

/** Constants for the launcher */
public class LauncherConstants {
  public static enum LauncherState {
    OFF(0),
    MANUAL(50),
    SCORING(50),
    PASSING(75); // TODO: find proper hood angles

    @Getter private final double hoodAngleRads;

    private LauncherState(double angle) {
      hoodAngleRads = angle;
    }
  }

  public static final int LAUNCHER_1_CAN_ID = 21; // TODO: double check
  public static final int LAUNCHER_2_CAN_ID = 22; // TODO: double check

  public static final double LAUNCHER_CURRENT_LIMIT = 20.0;
  public static final double LAUNCHER_GEARING = 1.0;
  public static final double DEFAULT_VELOCITY_SETPOINT_RPS = 60.0;

  public static final double ROLLER_RADIUS_METERS = Units.inchesToMeters(2.0);
  public static final double ROLLER_MASS_KG = Units.lbsToKilograms(0.7); // TODO: get weight
  public static final double ROLLER_CIRCUMFERENCE_METERS = Units.inchesToMeters(4.0 * Math.PI);
  public static final double LAUNCHER_HEIGHT_METERS =
      Units.inchesToMeters(22.5); // TODO: double check

  public static final TalonFXConfiguration LAUNCHER_CONFIG = new TalonFXConfiguration();
  public static final Slot0Configs LAUNCHER_CONFIG_SLOT0 = LAUNCHER_CONFIG.Slot0;

  public static final TalonFXConfiguration LAUNCHER_MOTOR_CONFIG() {
    LAUNCHER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    LAUNCHER_CONFIG.CurrentLimits.StatorCurrentLimit = LAUNCHER_CURRENT_LIMIT;

    LAUNCHER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    LAUNCHER_CONFIG.CurrentLimits.SupplyCurrentLimit = LAUNCHER_CURRENT_LIMIT;

    LAUNCHER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    LAUNCHER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    LAUNCHER_CONFIG_SLOT0.kP = 0.1;
    LAUNCHER_CONFIG_SLOT0.kI = 0.0;
    LAUNCHER_CONFIG_SLOT0.kD = 0.0;
    LAUNCHER_CONFIG_SLOT0.kS = 0.19;
    LAUNCHER_CONFIG_SLOT0.kV = 0.1;
    return LAUNCHER_CONFIG;
  }

  public static final int HOOD_SERVO_HUB_CAN_ID = 0; // TODO: set
  public static final ChannelId HOOD_1_ID = ChannelId.kChannelId0; // TODO: set
  public static final ChannelId HOOD_2_ID = ChannelId.kChannelId1; // TODO: set

  public static final int SERVO_COUNTER_CLOCKWISE_PULSE_WIDTH = 500; // TODO: double check
  public static final int SERVO_CLOCKWISE_PULSE_WIDTH = 2500; // TODO: double check
  public static final int SERVO_NO_MOVEMENT_PULSE_WIDTH = 1500;

  public static final double HOOD_GEARING = 25 / 12; // TODO: double check

  public static final int HIGH_ANGLE_LIMIT_SWITCH_CHANNEL = 0; // TODO: set
  public static final int LOW_ANGLE_LIMIT_SWITCH_CHANNEL = 0; // TODO: set
}
