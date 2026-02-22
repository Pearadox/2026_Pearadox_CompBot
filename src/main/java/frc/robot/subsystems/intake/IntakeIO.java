package frc.robot.subsystems.intake;

import frc.lib.drivers.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

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
