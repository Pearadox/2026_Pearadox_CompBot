// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import edu.wpi.first.math.util.Units;
import frc.lib.drivers.PearadoxTalonFX;
import org.littletonrobotics.junction.Logger;

/** Launcher IO's base class for TalonFX motors */
public abstract class LauncherIOTalonFX implements LauncherIO {

  protected final PearadoxTalonFX launcher1Leader;
  protected final PearadoxTalonFX launcher2Follower;

  protected final VelocityVoltage launcher1Control;
  protected final Follower launcher2Control;

  protected final ServoHub hoodServoHub;

  protected final ServoChannel hoodServo1;
  protected final ServoChannel hoodServo2;

  public LauncherIOTalonFX() {
    launcher1Leader =
        new PearadoxTalonFX(
            LauncherConstants.LAUNCHER_1_CAN_ID, LauncherConstants.LAUNCHER_MOTOR_CONFIG());
    launcher2Follower =
        new PearadoxTalonFX(
            LauncherConstants.LAUNCHER_2_CAN_ID, LauncherConstants.LAUNCHER_MOTOR_CONFIG());

    launcher1Control = new VelocityVoltage(0);
    launcher2Control = new Follower(launcher1Leader.getDeviceID(), MotorAlignmentValue.Opposed);

    hoodServoHub = new ServoHub(LauncherConstants.HOOD_SERVO_HUB_CAN_ID);

    hoodServo1 = hoodServoHub.getServoChannel(LauncherConstants.HOOD_1_ID);
    hoodServo1.setEnabled(true);
    hoodServo1.setPowered(true);

    hoodServo2 = hoodServoHub.getServoChannel(LauncherConstants.HOOD_2_ID);
    hoodServo2.setEnabled(true);
    hoodServo2.setPowered(true);
  }

  public void updateInputs(LauncherIOInputsAutoLogged inputs) {
    inputs.launcher1Data = launcher1Leader.getData();
    inputs.launcher2Data = launcher2Follower.getData();

    inputs.hoodServoHubVoltage = hoodServoHub.getDeviceVoltage();

    inputs.hoodServo1Position = pulseWidthtoAngularPosition(hoodServo1.getPulseWidth());
    inputs.hoodServo2Position = pulseWidthtoAngularPosition(hoodServo2.getPulseWidth());
  }

  public void runLauncherVelocity(double velocityRPS) {
    launcher1Leader.setControl(launcher1Control.withVelocity(velocityRPS));
    launcher2Follower.setControl(launcher2Control);
  }

  public void setHoodAngleRads(double angleRads) {
    if (angleRads < LauncherConstants.HOOD_MAX_ANGLE_RADS
        && angleRads > LauncherConstants.HOOD_MIN_ANGLE_RADS) {
      double angleRadsFromMinimum = angleRads - LauncherConstants.HOOD_MIN_ANGLE_RADS;

      double angularPosition =
          (Units.radiansToRotations(angleRadsFromMinimum) * LauncherConstants.HOOD_GEARING) / 5;

      hoodServo1.setPulseWidth(angularPositiontoPulseWidth(angularPosition));
      hoodServo2.setPulseWidth(
          LauncherConstants.HOOD_SERVO_MAX_PULSE_WIDTH
              - angularPositiontoPulseWidth(angularPosition));
    } else {
      Logger.recordOutput("HoodAngleOutOfRange", angleRads);
    }
  }

  public double pulseWidthtoAngularPosition(int pulseWidth) {
    return (pulseWidth - 500) / 2000;
  }

  public int angularPositiontoPulseWidth(double angularPosition) {
    return (int) (angularPosition * 2000) + 500;
  }
}
