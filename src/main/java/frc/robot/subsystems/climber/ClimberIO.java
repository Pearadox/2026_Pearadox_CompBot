package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.drivers.PearadoxTalonFX.MotorData;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public MotorData climberMotorData = new MotorData();

    }

    public default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

    public default void runPosition(double setpointRots) {}

    public default void zeroClimber() {}
}