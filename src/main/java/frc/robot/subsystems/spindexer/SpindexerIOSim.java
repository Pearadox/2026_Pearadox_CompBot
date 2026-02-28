// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Launcher IO's real implementation */
public class SpindexerIOSim extends SpindexerIOTalonFX {
  private SingleJointedArmSim spindexerPhysicsSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          SpindexerConstants.SPINDEXER_GEARING,
          SpindexerConstants.SPINDEXER_MOI,
          SpindexerConstants.SPINDEXER_RADIUS_METERS,
          Double.NEGATIVE_INFINITY,
          Double.POSITIVE_INFINITY,
          false,
          0);

  private TalonFXSimState spindexerSimState;

  public SpindexerIOSim() {
    super();
    spindexerSimState = spindexer.getSimState();
  }

  public void updateInputs(SpindexerIOInputsAutoLogged inputs) {
    super.updateInputs(inputs);
    updateSim();
  }

  public void updateSim() {
    spindexerSimState.setSupplyVoltage(12);
    spindexerPhysicsSim.setInputVoltage(spindexerSimState.getMotorVoltage());

    spindexerSimState.setRawRotorPosition(
        Units.radiansToRotations(spindexerPhysicsSim.getAngleRads()));
    spindexerSimState.setRotorVelocity(
        Units.radiansPerSecondToRotationsPerMinute(spindexerPhysicsSim.getVelocityRadPerSec())
            / 60);

    spindexerPhysicsSim.update(0.02);
  }
}
