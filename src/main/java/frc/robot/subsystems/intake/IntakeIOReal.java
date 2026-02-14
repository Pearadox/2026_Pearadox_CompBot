package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.util.Units;
import frc.lib.drivers.PearadoxTalonFX;

public class IntakeIOReal implements IntakeIO {
    private PearadoxTalonFX rollerMotor;
    private PearadoxTalonFX pivotMotor;

    private TalonFXConfiguration rollerConfigs;
    private TalonFXConfiguration pivotConfigs;

    public IntakeIOReal() {
        
        rollerMotor = new PearadoxTalonFX(IntakeConstants.ROLLER_ID, IntakeConstants.getRollerConfigTalonFX());
        rollerConfigs = IntakeConstants.getRollerConfigTalonFX();

        pivotMotor = new PearadoxTalonFX(IntakeConstants.PIVOT_ID, IntakeConstants.getPivotConfigTalonFX());
        pivotConfigs = IntakeConstants.getPivotConfigTalonFX();
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        inputs.rollerVoltage = rollerMotor.getMotorVoltage().getValueAsDouble();
        inputs.rollerVelocity = rollerMotor.getVelocity().getValueAsDouble();
        inputs.rollerSupplyCurrent = rollerMotor.getSupplyCurrent().getValueAsDouble();
        inputs.rollerStatorCurrent = rollerMotor.getStatorCurrent().getValueAsDouble();

        inputs.pivotVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotAngleRots = pivotMotor.getPosition().getValueAsDouble();
        inputs.pivotSupplyCurrent = pivotMotor.getSupplyCurrent().getValueAsDouble();
        inputs.pivotStatorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void runRollersVolts (double volts) {
        rollerMotor.setVoltage(volts);
    }

    @Override
    public void runPositionDegrees (double degrees) {
        pivotMotor.setControl(new PositionVoltage(Units.degreesToRotations(degrees)));
    }

}
