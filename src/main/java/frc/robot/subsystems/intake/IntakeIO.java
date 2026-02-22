package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.drivers.PearadoxTalonFX.MotorData;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public MotorData rollerMotorData = new MotorData();
        public MotorData pivotMotorData = new MotorData();
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void runRollersVolts(double volts) {}

    public default void runPositionDegrees(double degrees) {}

}
