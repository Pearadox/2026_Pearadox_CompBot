package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double positionRots = 0.0;
        public double appliedVolts = 0.0;
    }

    public default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

    public default void runPosition(double setpoint) {}
}