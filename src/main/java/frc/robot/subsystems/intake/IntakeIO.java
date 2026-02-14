package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double rollerVoltage = 0.0;
        public double rollerVelocity = 0.0;
        public double rollerSupplyCurrent = 0.0;
        public double rollerStatorCurrent = 0.0;

        public double pivotVoltage = 0.0;
        public double pivotAngleRots = 0.0;
        public double pivotSupplyCurrent = 0.0;
        public double pivotStatorCurrent = 0.0;
    }

    public default void updateInputs(IntakeIOInputsAutoLogged inputs) {}

    public default void runRollersVolts(double volts) {}

    public default void runPositionDegrees(double degrees) {}

}
