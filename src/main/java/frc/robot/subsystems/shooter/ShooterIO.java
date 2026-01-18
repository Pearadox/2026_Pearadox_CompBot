// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        public double rollerVelocity = 0.0;

        public double supplyCurrent = 0.0;
        public double statorCurrent = 0.0;

        public double rollerVoltage = 0.0;
    }

    public default void updateInputs(ShooterIOInputsAutoLogged inputs) {}

    public default void runVoltage(double voltage) {}

}
