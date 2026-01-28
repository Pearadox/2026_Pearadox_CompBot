package frc.robot.util;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class DriveHelpers {
     public static Rotation2d findClosestCorner(Supplier<Pose2d> robotPoseSupplier) {
        double currentDegrees = robotPoseSupplier.get().getRotation().getDegrees();
        double[] robotCornersInDegrees = {45, 45 + 90, 45 + 180, 45 + 270};
        double minError = Double.MAX_VALUE;
        double closest = robotCornersInDegrees[0];

        for (double c : robotCornersInDegrees) {

            double error = Math.abs(Math.IEEEremainder(c - currentDegrees, 360));
            if (error < minError) {
                minError = error;
                closest = c;
            }
        }
        return new Rotation2d(Units.degreesToRadians(closest));
    }

}
