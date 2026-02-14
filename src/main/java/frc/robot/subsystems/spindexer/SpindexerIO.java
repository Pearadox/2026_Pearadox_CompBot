package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

    @AutoLog
    public static class SpindexerIOInputs {
        public double spindexerVelocity = 0.0;
        public double spindexerVoltage = 0.0;
        public double spindexerStatorCurrent = 0.0;
        public double spindexerSupplyCurrent = 0.0;
    }

    public default void updateInputs(SpindexerIOInputsAutoLogged inputs) {}

    public default void runSpindexerVoltage(double voltage) {}
}
