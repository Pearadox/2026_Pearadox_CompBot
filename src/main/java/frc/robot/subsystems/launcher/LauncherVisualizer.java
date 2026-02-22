// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class LauncherVisualizer {
  private final LoggedMechanism2d mech2d = new LoggedMechanism2d(3, 3);

  private final LoggedMechanismRoot2d root = mech2d.getRoot("Roller", 1, 1.5);

  private final Color8Bit staticColor = new Color8Bit(Color.kDarkSlateGray);
  private final Color8Bit activeColor = new Color8Bit(Color.kSeaGreen);
  private final Color8Bit visibilityLigament = new Color8Bit(Color.kGoldenrod);
  private LoggedMechanismLigament2d[] wheel = createWheel(40);

  private LoggedMechanismLigament2d hood =
      root.append(
          new LoggedMechanismLigament2d("Hood", 1, 20, 10, new Color8Bit(Color.kDimGray)));

  private double previousRollerAngle = 0.0;
  private double rollerAngleDeg = 0.0;
  private double hoodAngleDeg = 0.0;

  private static final LauncherVisualizer instance = new LauncherVisualizer();

  public static LauncherVisualizer getInstance() {
    return instance;
  }

  private LauncherVisualizer() {}

  public void periodic() {
    SmartDashboard.putData("Launcher Sim", mech2d);
  }

  public LoggedMechanismLigament2d[] createWheel(int size) {
    LoggedMechanismLigament2d[] wheel = new LoggedMechanismLigament2d[size];
    for (int i = 0; i < size; i++) {
      wheel[i] =
          root.append(
              new LoggedMechanismLigament2d(
                  "Launcher Wheel Segment" + Integer.toString(i),
                  0.2,
                  360 / size * (i + 1),
                  4,
                  (i == size - 1 ? visibilityLigament : staticColor)));
    }
    return wheel;
  }

  public void updateRollerPositionDeg(double angleDeg) {
    rollerAngleDeg = angleDeg;
    for (int i = 0; i < wheel.length; i++) {
      wheel[i].setAngle(rollerAngleDeg + (360 / wheel.length * (i + 1)));
      if (i != wheel.length - 1) {
        if (rollerAngleDeg != previousRollerAngle) {
          wheel[i].setColor(activeColor);
        } else {
          wheel[i].setColor(staticColor);
        }
      }
    }
    previousRollerAngle = rollerAngleDeg;
  }

  public void updateHoodPositionDeg(double angleDeg) {
    hoodAngleDeg = angleDeg;
    hood.setAngle(hoodAngleDeg);
  }
}
