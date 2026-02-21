package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.lib.drivers.PearadoxTalonFX;

public class SpindexerIOReal implements SpindexerIO {
  private PearadoxTalonFX spindexer;
  private TalonFXConfiguration spindexerConfig;

  public SpindexerIOReal() {
    spindexerConfig = SpindexerConstants.spindexerConfig();

    spindexer = new PearadoxTalonFX(SpindexerConstants.SPINDEXER_MOTOR_ID, spindexerConfig);
  }

  @Override
  public void updateInputs(SpindexerIOInputsAutoLogged inputs) {
    inputs.spindexerVelocity = spindexer.getVelocity().getValueAsDouble();
    inputs.spindexerVoltage = spindexer.getMotorVoltage().getValueAsDouble();
    inputs.spindexerStatorCurrent = spindexer.getStatorCurrent().getValueAsDouble();
    inputs.spindexerSupplyCurrent = spindexer.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void runSpindexerVoltage(double voltage) {
    spindexer.setControl(new VoltageOut(voltage));
  }
}
