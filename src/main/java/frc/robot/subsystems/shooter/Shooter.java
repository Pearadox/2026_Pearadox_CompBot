// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;
import frc.robot.util.SmarterDashboard;

public class Shooter extends SubsystemBase {
    /** Creates a new Shooter. */

    private ShooterIO io;

    private ShooterState shooterState = ShooterState.OFF;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        SmarterDashboard.putString("Shooter/ShooterState", shooterState.toString());
        SmarterDashboard.putNumber("Shooter/VoltageOut", inputs.rollerVoltage);
        SmarterDashboard.putNumber("Shooter/Velocity", inputs.rollerVelocity);
        SmarterDashboard.putNumber("Shooter/Voltage-Adjust", shooterState.getAdjust());

        io.runVoltage(shooterState.getVoltage());
    }

    public void setManual() {
        shooterState = ShooterState.MANUAL;
    }
    
    public void setAutoScore() {
        shooterState = ShooterState.AUTO_SCORE;
    }

    public void setAutoPass() {
        shooterState = ShooterState.AUTO_PASS;
    }

    public void setOff() {
        shooterState = ShooterState.OFF;
    }

    public void adjustSpeed(double adjustBy) {
        shooterState.adjustVoltage(adjustBy);
    }

    public void resetAdjust() {
        shooterState.resetAdjust();
    }

}
