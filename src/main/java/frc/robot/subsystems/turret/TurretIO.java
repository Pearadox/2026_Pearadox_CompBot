package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.drivers.PearadoxTalonFX.MotorData;

public interface TurretIO {
    @AutoLog
    static class TurretIOInputs {
        public MotorData turretData = new MotorData();
    }

    default void updateInputs(TurretIOInputs inputs) {}

    default void runPosition(double setpointRots, double ffVolts) {}
}
