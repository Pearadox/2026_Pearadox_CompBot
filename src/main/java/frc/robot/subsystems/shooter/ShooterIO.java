// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        public double shooterVelocity = 0.0;
        public double shooterVoltage = 0.0;

        public double shooterSupplyCurrent = 0.0;
        public double shooterStatorCurrent = 0.0;

        public double transportVelocity = 0.0;
        public double transportVoltage = 0.0;

        public double transportSupplyCurrent = 0.0;
        public double transportStatorCurrent = 0.0;
    }

    public default void updateInputs(ShooterIOInputsAutoLogged inputs) {}

    public default void runShooterVoltage(double voltage) {}

    public default void runTransportVoltage(double voltage) {}

}
