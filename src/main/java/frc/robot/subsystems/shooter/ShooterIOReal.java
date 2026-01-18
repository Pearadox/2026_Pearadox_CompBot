// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.lib.drivers.PearadoxTalonFX;

/** Add your docs here. */
public class ShooterIOReal implements ShooterIO {

    private PearadoxTalonFX roller1;
    private PearadoxTalonFX roller2;

    private TalonFXConfiguration rollerConfigs;

    public ShooterIOReal() {
        roller1 = new PearadoxTalonFX(
            ShooterConstants.ROLLER_1_CAN_ID,
            ShooterConstants.NEUTRAL_MODE,
            ShooterConstants.CURRENT_LIMIT,
            ShooterConstants.INVERTED);
        roller2 = new PearadoxTalonFX(
            ShooterConstants.ROLLER_2_CAN_ID,
            ShooterConstants.NEUTRAL_MODE,
            ShooterConstants.CURRENT_LIMIT,
            ShooterConstants.INVERTED);

        

        rollerConfigs = new TalonFXConfiguration();
        rollerConfigs.Slot0 = ShooterConstants.get_shooter_config();
    }

    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        inputs.rollerVelocity = roller1.getVelocity().getValueAsDouble();

        inputs.statorCurrent = roller1.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrent = roller1.getSupplyCurrent().getValueAsDouble();

        inputs.rollerVoltage = roller1.getMotorVoltage().getValueAsDouble();
    }

    public void runVoltage(double voltage) {
        roller1.setControl(new VoltageOut(voltage));
        roller2.setControl(new Follower(ShooterConstants.ROLLER_1_CAN_ID, MotorAlignmentValue.Opposed));
    }

}
