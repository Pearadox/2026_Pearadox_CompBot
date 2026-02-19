package frc.robot.subsystems.spindexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import frc.lib.drivers.PearadoxTalonFX;

public class SpindexerIOReal implements SpindexerIO {
    private PearadoxTalonFX spindexer;
    private TalonFXConfiguration spindexerConfig;

    public SpindexerIOReal() {
        spindexerConfig = SpindexerConstants.spindexerConfig();

        spindexer = new PearadoxTalonFX(
            SpindexerConstants.SPINDEXER_MOTOR_ID,
            spindexerConfig
        );
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
