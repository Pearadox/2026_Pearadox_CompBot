// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.lib.drivers.PearadoxTalonFX;

/** Add your docs here. */
public class FeederIOReal implements FeederIO {

  private PearadoxTalonFX feeder;

  private VoltageOut feederControl;

  public FeederIOReal() {
    feeder =
        new PearadoxTalonFX(FeederConstants.FEEDER_CAN_ID, FeederConstants.FEEDER_MOTOR_CONFIG());
    feederControl = new VoltageOut(0.0);
  }

  public void updateInputs(FeederIOInputsAutoLogged inputs) {
    inputs.feederData = feeder.getData();
  }

  public void runFeederVoltage(double voltage) {
    feeder.setControl(feederControl.withOutput(voltage));
  }
}
