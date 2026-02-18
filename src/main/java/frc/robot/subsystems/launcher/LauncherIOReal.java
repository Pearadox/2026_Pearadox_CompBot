// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;

import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;
import frc.robot.util.PearadoxTalonFX;

/** Launcher IO's real implementation */
public class LauncherIOReal implements LauncherIO {

    private PearadoxTalonFX launcher1Leader;
    private PearadoxTalonFX launcher2Follower;

    private VelocityVoltage launcher1Control;
    private Follower launcher2Control;

    private ServoHub hoodServoHub;

    private ServoChannel hoodServo1;
    private ServoChannel hoodServo2;

    private DigitalInput highAngleLimitSwitch;
    private DigitalInput lowAngleLimitSwitch;

    public LauncherIOReal() {
        launcher1Leader = new PearadoxTalonFX(LauncherConstants.LAUNCHER_1_CAN_ID, LauncherConstants.LAUNCHER_MOTOR_CONFIG());
        launcher2Follower = new PearadoxTalonFX(LauncherConstants.LAUNCHER_2_CAN_ID, LauncherConstants.LAUNCHER_MOTOR_CONFIG());

        launcher1Control = new VelocityVoltage(0);
        launcher2Control = new Follower(launcher1Leader.getDeviceID(), MotorAlignmentValue.Opposed);
        
        BaseStatusSignal.setUpdateFrequencyForAll(
            Constants.UPDATE_FREQ_SEC,
            launcher1Leader.getVelocity(),
            launcher1Leader.getMotorVoltage(),
            launcher2Follower.getVelocity(),
            launcher2Follower.getMotorVoltage()
            );

        hoodServoHub = new ServoHub(LauncherConstants.HOOD_SERVO_HUB_CAN_ID);

        hoodServo1 = hoodServoHub.getServoChannel(LauncherConstants.HOOD_1_ID);
        hoodServo1.setEnabled(true);
        hoodServo1.setPowered(true);
        hoodServo1.setPulseWidth(1500);
        
        hoodServo2 = hoodServoHub.getServoChannel(LauncherConstants.HOOD_2_ID);
        hoodServo2.setEnabled(true);
        hoodServo2.setPowered(true);
        hoodServo2.setPulseWidth(1500);
        
        highAngleLimitSwitch = new DigitalInput(LauncherConstants.HIGH_ANGLE_LIMIT_SWITCH_CHANNEL);
        lowAngleLimitSwitch = new DigitalInput(LauncherConstants.LOW_ANGLE_LIMIT_SWITCH_CHANNEL);

    }

    public void updateInputs(LauncherIOInputsAutoLogged inputs) {
        inputs.launcher1Data = launcher1Leader.getData();
        inputs.launcher2Data = launcher2Follower.getData();

        inputs.hoodServoHubVoltage = hoodServoHub.getDeviceVoltage();
        inputs.hoodServo1PulseWidth = hoodServo1.getPulseWidth();
        inputs.hoodServo2PulseWidth = hoodServo2.getPulseWidth();

        inputs.limitSwitchPressed = (lowAngleLimitSwitch.get() || highAngleLimitSwitch.get());
    }

    public void runLauncherVelocity(double velocityRPS) {
        launcher1Leader.setControl(launcher1Control.withVelocity(velocityRPS));
        launcher2Follower.setControl(launcher2Control);
    }

    public void setHoodAngle(boolean isPassing) {
        if (isPassing) {
            if (highAngleLimitSwitch.get()) {
                hoodServo1.setPulseWidth(LauncherConstants.SERVO_CLOCKWISE_PULSE_WIDTH);
                hoodServo2.setPulseWidth(LauncherConstants.SERVO_COUNTER_CLOCKWISE_PULSE_WIDTH);
            } else {
                hoodServo1.setPulseWidth(LauncherConstants.SERVO_NO_MOVEMENT_PULSE_WIDTH);
                hoodServo2.setPulseWidth(LauncherConstants.SERVO_NO_MOVEMENT_PULSE_WIDTH);
            }
        } else {
            if (lowAngleLimitSwitch.get()) {
                hoodServo1.setPulseWidth(LauncherConstants.SERVO_COUNTER_CLOCKWISE_PULSE_WIDTH);
                hoodServo2.setPulseWidth(LauncherConstants.SERVO_CLOCKWISE_PULSE_WIDTH);
            } else {
                hoodServo1.setPulseWidth(LauncherConstants.SERVO_NO_MOVEMENT_PULSE_WIDTH);
                hoodServo2.setPulseWidth(LauncherConstants.SERVO_NO_MOVEMENT_PULSE_WIDTH);
            }
        }
    }

}
