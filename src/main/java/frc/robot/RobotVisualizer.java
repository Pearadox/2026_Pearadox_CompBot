package frc.robot;

import static frc.robot.Constants.VisualizerConstants.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
    private final DoubleSupplier turretYawSupplier;
    private final DoubleSupplier hoodAngleSupplier;
    private final DoubleSupplier spindexerYawSupplier;
    private final DoubleSupplier intakeAngleSupplier;
    private final DoubleSupplier climberDisplacementSupplier;

    @Getter
    private Transform3d hoodTransform = Transform3d.kZero;

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
        double hoodPitch = hoodAngleSupplier.getAsDouble();
        double spindexerYaw = spindexerYawSupplier.getAsDouble();
        double intakeRoll = intakeAngleSupplier.getAsDouble();
        double climberDisplacement = climberDisplacementSupplier.getAsDouble();

        Transform3d turret = new Transform3d(Z0_ZERO, new Rotation3d(0, 0, -turretYaw + TURRET_STARTING_ANGLE));
        hoodTransform = turret.plus(new Transform3d(Z1_OFFSET, new Rotation3d(-hoodPitch + HOOD_STARTING_ANGLE, 0, 0)));
        Transform3d spindexer = new Transform3d(Z2_ZERO, new Rotation3d(0, 0, -spindexerYaw));
        Transform3d intake = new Transform3d(Z3_ZERO, new Rotation3d(0, -intakeRoll + INTAKE_STARTING_ANGLE, 0));
        Transform3d intakeGravityRamp =
                intake.plus(new Transform3d(Z4_OFFSET, new Rotation3d(0, getGravityRampAngle(intakeRoll), 0)));
        Transform3d climber = new Transform3d(
                new Translation3d(0, 0, climberDisplacement - CLIMBER_MAX_DISPLACEMENT), Rotation3d.kZero);

        Logger.recordOutput(
                "RobotVisualizer/Components",
                new Transform3d[] {turret, hoodTransform, spindexer, intake, intakeGravityRamp, climber});
    }

    private static double getGravityRampAngle(double intakeAngle) {
        double intakeAngleDegs = Units.radiansToDegrees(intakeAngle) % 360;

        // the ramp wants to point downwards (with gravity) but is
        // constrained (with rubber bands) from being more than
        // 39 degrees away from the intake
        double fieldRelativeAngle = Math.min(intakeAngleDegs - 180 + 39, -90);

        // return an angle relative to the intake
        // adding 180 bc the CAD was exported with the ramp facing down
        // and negative of the side of the robot it's on
        return -(Units.degreesToRadians(fieldRelativeAngle - intakeAngleDegs + 180));
    }
}
