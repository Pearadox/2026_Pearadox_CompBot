package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import frc.lib.drivers.PearadoxTalonFX;

public abstract class ClimberIOTalonFX implements ClimberIO {
    protected final PearadoxTalonFX climber;
    protected final PositionVoltage climberControl;

    public ClimberIOTalonFX() {
        climber = new PearadoxTalonFX(
            ClimberConstants.CLIMBER_MOTOR_ID, ClimberConstants.getClimberConfigTalonFX());
        
        climberControl = new PositionVoltage(0); 
    }

    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs) {
        inputs.climberMotorData = climber.getData();
    }
    
    @Override
    public void runPosition(double setpointRots) {
        climber.setControl(climberControl.withPosition(setpointRots));
    }

    @Override
    public void zeroClimber() {
        climber.setPosition(0.0);
    }
}