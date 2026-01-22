// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.lib.drivers.PearadoxTalonFX;

/** Add your docs here. */
public class ShooterIOReal implements ShooterIO {

    private PearadoxTalonFX shooter1Leader;
    private PearadoxTalonFX shooter2Follower;

    private TalonFXConfiguration shooterConfigs;

    private PearadoxTalonFX transport;

    public ShooterIOReal() {
        shooter1Leader = new PearadoxTalonFX(
            ShooterConstants.SHOOTER_1_CAN_ID,
            ShooterConstants.rollerConfig()
        );
        shooter2Follower = new PearadoxTalonFX(
            ShooterConstants.SHOOTER_2_CAN_ID,
            ShooterConstants.rollerConfig()
        );

        shooterConfigs = new TalonFXConfiguration();
        shooterConfigs.Slot0 = ShooterConstants.shooterSlot0config();


        transport = new PearadoxTalonFX(
            ShooterConstants.TRANSPORT_CAN_ID,
            ShooterConstants.TRANPORT_NEUTRAL_MODE,
            ShooterConstants.TRANSPORT_CURRENT_LIMIT,
            ShooterConstants.TRANSPORT_INVERTED
        );
    }

    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        inputs.shooterVelocity = shooter1Leader.getVelocity().getValueAsDouble();
        inputs.shooterVoltage = shooter1Leader.getMotorVoltage().getValueAsDouble();

        inputs.shooterStatorCurrent = shooter1Leader.getStatorCurrent().getValueAsDouble();
        inputs.shooterSupplyCurrent = shooter1Leader.getSupplyCurrent().getValueAsDouble();

        inputs.transportVelocity = transport.getVelocity().getValueAsDouble();
        inputs.transportVoltage = transport.getMotorVoltage().getValueAsDouble();

        inputs.transportStatorCurrent = transport.getStatorCurrent().getValueAsDouble();
        inputs.transportSupplyCurrent = transport.getSupplyCurrent().getValueAsDouble();
    }

    public void runShooterVoltage(double voltage) {
        shooter1Leader.setControl(new VoltageOut(voltage));
        shooter2Follower.setControl(new Follower(ShooterConstants.SHOOTER_1_CAN_ID, MotorAlignmentValue.Opposed));
    }
    
    public void runShooterVelocity(double velocity) {
        shooter1Leader.setControl(new VelocityVoltage(velocity));
        shooter2Follower.setControl(new Follower(ShooterConstants.SHOOTER_1_CAN_ID, MotorAlignmentValue.Opposed));
    }

    public void runTransportVoltage(double voltage) {
        transport.setVoltage(voltage);
    }

}
