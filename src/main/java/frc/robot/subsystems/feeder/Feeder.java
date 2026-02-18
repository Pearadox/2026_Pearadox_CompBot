// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

	private final FeederIO io;

	private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

	/** Creates a new Feeder. */
	public Feeder(FeederIO io) {
		this.io = io;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		io.updateInputs(inputs);
		Logger.processInputs("FeederInputs", inputs);
	}

	public void launch() {
		io.runFeederVoltage(FeederConstants.FEEDER_ACTIVE_VOLTAGE);
	}

	public void stopLaunch() {
		io.runFeederVoltage(0.0);
	}
}
