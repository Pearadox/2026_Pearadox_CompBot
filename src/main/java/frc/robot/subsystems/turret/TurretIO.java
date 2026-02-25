package frc.robot.subsystems.turret;

import frc.lib.drivers.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  static class TurretIOInputs {
    public MotorData turretData = new MotorData();
  }

  default void updateInputs(TurretIOInputs inputs) {}

  default void runPosition(double setpointRots, double ffVolts) {}
}
