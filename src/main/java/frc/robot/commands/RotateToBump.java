package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class RotateToBump extends Command {

    private Drive drive;
    private ProfiledPIDController rotationController;
    private double targetAngleRadians;
    private Supplier<Pose2d> robotPoseSupplier;

    private static final LoggedTunableNumber rotkP = new LoggedTunableNumber("RotateToBump/kP", AlignConstants.ROT_kP);
    private static final LoggedTunableNumber rotkI = new LoggedTunableNumber("RotateToBump/kI", AlignConstants.ROT_kI); 
    private static final LoggedTunableNumber rotkD = new LoggedTunableNumber("RotateToBump/kD", AlignConstants.ROT_kD); 

    private static final LoggedTunableNumber maxRotVel = new LoggedTunableNumber("RotateToBump/maxRotVel", AlignConstants.MAX_ROT_VELOCITY);
    private static final LoggedTunableNumber maxRotAcc = new LoggedTunableNumber("RotateToBump/maxRotAcc", AlignConstants.MAX_ROT_ACCELERATION);

    // override right joystick controls
    // find closest corner of robot that aligns with the bump

    public RotateToBump(Drive drive, Supplier<Pose2d> robotPoseSupplier) {
        this.drive = drive;
        this.robotPoseSupplier = robotPoseSupplier;

        rotationController = new ProfiledPIDController(
            rotkP.getAsDouble(),
            rotkI.getAsDouble(),
            rotkD.getAsDouble(),
            new TrapezoidProfile.Constraints(
                maxRotVel.getAsDouble(),
                maxRotVel.getAsDouble()
                )
            );

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        double currentAngleDegrees =
            robotPoseSupplier.get().getRotation().getDegrees();
        double closestCornerDegrees =
            findClosestCorner(currentAngleDegrees);
        targetAngleRadians =
            Units.degreesToRadians(closestCornerDegrees);
        double currentOmega =
            drive.getChassisSpeeds().omegaRadiansPerSecond;
        rotationController.reset(
            robotPoseSupplier.get().getRotation().getRadians(),
            currentOmega);
    }

    @Override
    public void execute() {
        double currentAngleRadians =
            robotPoseSupplier.get().getRotation().getRadians();
        double omegaOutput =
            rotationController.calculate(
                currentAngleRadians,
                targetAngleRadians);
        drive.runVelocity(
            new ChassisSpeeds(0,0,omegaOutput));
    }

    @Override
    public boolean isFinished() {
        return rotationController.atGoal();
    }

    @Override
    public void end(boolean interrputed) {
        drive.stop();
    }

    private double findClosestCorner(double currentDegrees) {
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
        return closest;
    }

}
