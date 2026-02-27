// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Utility class to better visualize a circle in simulation, whether for a roller, flywheel, or
 * something else idk
 */
public class CircleSim {

  private final LoggedMechanismRoot2d root;

  private final LoggedMechanismLigament2d[] circle;

  private final Color8Bit staticColor;
  private final Color8Bit activeColor;
  private final Color8Bit visibilityColor;

  /**
   * Creates an entire circle mechanism to be placed into the simulation window.
   *
   * @param root the root (or a new root) that the circle is added to.
   * @param segmentCount the number of segments on the circle.
   * @param radius the radius of the circle.
   * @param staticColor the color of the circle when it is not rotating.
   * @param activeColor the color of the circle when it is rotating.
   * @param visibilityColor the color of the segment that makes it easier to see the direction of
   *     rotation.
   */
  public CircleSim(
      LoggedMechanismRoot2d root,
      int segmentCount,
      double radius,
      Color8Bit staticColor,
      Color8Bit activeColor,
      Color8Bit visibilityColor) {

    this.root = root;

    circle = new LoggedMechanismLigament2d[segmentCount];

    this.staticColor = staticColor;
    this.activeColor = activeColor;
    this.visibilityColor = visibilityColor;

    for (int i = 0; i < circle.length - 1; i++) {
      circle[i] =
          this.root.append(
              new LoggedMechanismLigament2d(
                  this.root.getName() + " segment " + (i + 1),
                  radius,
                  360 * (i + 1) / circle.length,
                  6,
                  this.staticColor));
    }

    circle[circle.length - 1] =
        this.root.append(
            new LoggedMechanismLigament2d(
                this.root.getName() + " visibility segment", radius, 0, 6, this.visibilityColor));
  }

  /**
   * @return the root used/created for the circle.
   */
  public LoggedMechanismRoot2d getRoot() {
    return root;
  }

  /**
   * Updates the angles of the spokes of the circle and sets their color based on their movement.
   *
   * @param newAngleDeg the angle to set the spokes to in DEGREES
   * @param previousAngleDeg the previous angle, used to determine color. no specific unit as long
   *     as it's consistent
   */
  public void updateAngleDeg(double newAngleDeg, double previousAngleDeg) {
    for (int i = 0; i < circle.length - 1; i++) {
      circle[i].setAngle(newAngleDeg + 360 / circle.length * (i + 1));
      if (Math.abs(newAngleDeg - previousAngleDeg) < 5) {
        circle[i].setColor(staticColor);
      } else {
        circle[i].setColor(activeColor);
      }
    }
    circle[circle.length - 1].setAngle(newAngleDeg);
  }
}
