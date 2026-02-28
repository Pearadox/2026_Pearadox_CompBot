package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.lib.drivers.PearadoxTalonFX;

public abstract class SpindexerIOTalonFX implements SpindexerIO {
  protected PearadoxTalonFX spindexer;
  private TalonFXConfiguration spindexerConfig;

  public SpindexerIOTalonFX() {
    spindexerConfig = SpindexerConstants.spindexerConfig();

    spindexer = new PearadoxTalonFX(SpindexerConstants.SPINDEXER_MOTOR_ID, spindexerConfig);
  }

  @Override
  public void updateInputs(SpindexerIOInputsAutoLogged inputs) {
    inputs.spindexerMotorData = spindexer.getData();
  }

  @Override
  public void runSpindexerVoltage(double voltage) {
    spindexer.setControl(new VoltageOut(voltage));
  }
}
