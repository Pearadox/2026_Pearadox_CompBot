// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.FeederConstants.FeederState;
import frc.robot.subsystems.feeder.FeederConstants.StateConfig;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  private final FeederIO io;
  private final CANrange canRange = new CANrange(4);
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private Debouncer canRangeDebouncer = new Debouncer(0.125, DebounceType.kFalling);
  private FeederState feederState = FeederState.RUNNING;

  /** Creates a new Feeder. */
  public Feeder(FeederIO io) {
    this.io = io;
    canRange.getConfigurator().apply(FeederConstants.canRangeConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("FeederInputs", inputs);
    Logger.recordOutput("Feeder/CanRange Distance", canRange.getDistance().getValueAsDouble());
    Logger.recordOutput("Feeder/CanRange FuelIsDetected", canRange.getIsDetected().getValue());
    io.runFeederVoltage(StateConfig.SPINDEXER_STATE_MAP.get(feederState).voltage());
  }

  public void setStopped() {
    feederState = FeederState.STOPPED;
  }

  public void setRunning() {
    feederState = FeederState.RUNNING;
  }

  public boolean isDetectedDebounced() {
    return canRangeDebouncer.calculate(canRange.getIsDetected().getValue());
  }

  // public void launch() {
  //   io.runFeederVoltage(FeederConstants.FEEDER_ACTIVE_VOLTAGE);
  // }

  // public void stopLaunch() {
  //   io.runFeederVoltage(0.0);
  // }
}
