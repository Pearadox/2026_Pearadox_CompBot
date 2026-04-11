package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.VisualizerConstants;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
  private final DoubleSupplier turretYawSupplier;
  private final DoubleSupplier hoodAngleSupplier;
  private final DoubleSupplier spindexerYawSupplier;
  private final DoubleSupplier intakeAngleSupplier;
  private final DoubleSupplier climberDisplacementSupplier;

  @Getter private Transform3d hoodTransform = Transform3d.kZero;
  @Getter private Transform3d llTransform = Transform3d.kZero;

  public RobotVisualizer(
      DoubleSupplier turretYawSupplier,
      DoubleSupplier hoodAngleSupplier,
      DoubleSupplier spindexerYawSupplier,
      DoubleSupplier intakeAngleSupplier,
      DoubleSupplier climberDisplacementSupplier) {
    this.turretYawSupplier = turretYawSupplier;
    this.hoodAngleSupplier = hoodAngleSupplier;
    this.spindexerYawSupplier = spindexerYawSupplier;
    this.intakeAngleSupplier = intakeAngleSupplier;
    this.climberDisplacementSupplier = climberDisplacementSupplier;
  }

  public void periodic() {
    double turretYaw = turretYawSupplier.getAsDouble();
    double hoodPitch =
        // 2 * Math.sin(RobotController.getFPGATime() / 1e6);
         hoodAngleSupplier.getAsDouble();
    double intakeRoll =
        Math.sin(RobotController.getFPGATime() / 1e6); // intakeAngleSupplier.getAsDouble();

    Transform3d turret =
        new Transform3d(
            VisualizerConstants.MODEL0_ZERO,
            new Rotation3d(0, 0, -turretYaw + VisualizerConstants.TURRET_STARTING_ANGLE));
    hoodTransform =
        turret.plus(
            new Transform3d(
                VisualizerConstants.MODEL1_OFFSET,
                new Rotation3d(-hoodPitch + VisualizerConstants.HOOD_STARTING_ANGLE, 0, 0)));
    Transform3d intake =
        new Transform3d(
            VisualizerConstants.MODEL2_ZERO,
            new Rotation3d(0, -intakeRoll + VisualizerConstants.INTAKE_STARTING_ANGLE, 0));

    Transform3d hopper =
        new Transform3d(
            new Translation3d(
                Math.pow(Math.sin(RobotController.getFPGATime() / 1e6), 2) * -VisualizerConstants.HOPPER_EXTENSION, 0, 0),
            Rotation3d.kZero);

    Logger.recordOutput(
        "RobotVisualizer/Components", new Transform3d[] {turret, hoodTransform, intake, hopper});

    llTransform =
        turret.plus(
            new Transform3d(
                0,
                VisualizerConstants.LIMELIGHT_TO_CENTER_OF_TURRET,
                VisualizerConstants.LIMELIGHT_HEIGHT,
                new Rotation3d(0, Math.toRadians(-20), Math.toRadians(90))));

    Logger.recordOutput("RobotVisualizer/LL", llTransform);
  }
}
