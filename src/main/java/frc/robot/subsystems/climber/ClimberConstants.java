// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import lombok.Getter;

/** Add your docs here. */
public class ClimberConstants {
  public static enum ClimberState {
    RAISED(0.0),
    LOWERED(50.0);

    @Getter private final double climberPositionRotations;

    private ClimberState(double rots) {
      climberPositionRotations = rots;
    }
  }

  public static final int CLIMBER_MOTOR_ID = 0; // TODO: find id
  public static final TalonFXConfiguration CLIMBER_CONFIG = new TalonFXConfiguration();
  public static final Slot0Configs CLIMBER_SLOT0_CONFIGS = CLIMBER_CONFIG.Slot0;
  public static final double CLIMBER_GEARING = 25.0; // TODO: find gearing
  public static final double CLIMBER_UP_SETPOINT_RADS = 0; // TODO: find setpoint in radians
  public static final double CLIMBER_DOWN_SETPOINT_RADS = 0; // TODO: find setpoint in radians
  public static final double CLIMBER_DRUM_RADIUS_METERS = Units.inchesToMeters(0.25);
  public static final double CLIMBER_MAX_HEIGHT_METERS = Units.inchesToMeters(25.0);
  public static final double CLIMBER_MIN_HEIGHT_METERS = Units.inchesToMeters(4.0);
  public static final double CLIMBER_DRUM_CIRCUMFERENCE = 2 * CLIMBER_DRUM_RADIUS_METERS * Math.PI;

  public static final TalonFXConfiguration getClimberConfigTalonFX() {
    CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLimit = 20;
    CLIMBER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    CLIMBER_CONFIG.CurrentLimits.StatorCurrentLimit = 20;

    CLIMBER_SLOT0_CONFIGS.kP = 0.1; // TODO: find kP
    CLIMBER_SLOT0_CONFIGS.kI = 0; // TODO: find kI
    CLIMBER_SLOT0_CONFIGS.kD = 0; // TODO: find kD

    CLIMBER_CONFIG.MotorOutput.Inverted =
        InvertedValue.CounterClockwise_Positive; // TODO: find if inverted
    CLIMBER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    return CLIMBER_CONFIG;
  }
}
