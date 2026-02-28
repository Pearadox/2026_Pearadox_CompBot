package frc.lib.drivers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.Hub;
import frc.robot.Constants.FieldConstants.LinesHorizontal;
import frc.robot.Constants.FieldConstants.LinesVertical;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherConstants;
import frc.robot.util.SmarterDashboard;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class MovingShotSolver {

  private static final double g = 9.81; // gravity constant in m/s^2

  private static Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

  private static double hubXMeters =
      alliance == Alliance.Red ? Hub.topCenterPointRed.getX() : Hub.topCenterPointBlue.getX();
  private static double hubYMeters =
      alliance == Alliance.Red ? Hub.topCenterPointRed.getY() : Hub.topCenterPointBlue.getY();

  private static double neutralZoneTargetX =
      alliance == Alliance.Red ? LinesVertical.redHubCenterX : LinesVertical.blueHubCenterX;
  private static double[] neutralZoneTargetYs = {
    LinesHorizontal.leftBumpStart - 0.25, LinesHorizontal.rightBumpEnd + 0.25
  };
  private static double neutralZoneHeightMeters = 2.0; // arbitrary for now

  private static double hubHeightMeters = Hub.height;
  private static double shooterHeightMeters = Units.inchesToMeters(22.5);
  private static double hoodAngleRadians = Units.degreesToRadians(65);

  private static Pose2d targetPose = new Pose2d();

  private static double goalXMeters = hubXMeters;
  private static double goalYMeters = hubYMeters;
  private static double goalHeightMeters = hubHeightMeters;

  private static boolean isInsideNeutralZone = false;

  private static final double MPSToRPSConversion = 
            LauncherConstants.LAUNCHER_GEARING / LauncherConstants.ROLLER_CIRCUMFERENCE_METERS;

  public static class ShotSolution {

    public final double timeOfFlight; // seconds
    public final double shooterSpeedRPS; // rot / s
    public final Rotation2d turretAngleRot2d; // field-relative turret angle
    public final boolean isInsideNeutralZone;

    public ShotSolution(
        double time, double speed, Rotation2d turretAngle, boolean isInsideNeutralZone) {
      this.timeOfFlight = time;
      this.shooterSpeedRPS = speed;
      this.turretAngleRot2d = turretAngle;
      this.isInsideNeutralZone = isInsideNeutralZone;
    }

    public double getTimeOfFlight() {
      return timeOfFlight;
    }

    public Rotation2d getTurretAngleRot2d() {
      return turretAngleRot2d;
    }

    public double getShooterSpeedRPS() {
      return shooterSpeedRPS;
    }

    public boolean getIsInsideNeutralZone() {
      return isInsideNeutralZone;
    }
  }

  public static double findClosestNeutralZoneSide(Pose2d curPose) {
    return Math.abs(curPose.getY() - neutralZoneTargetYs[0])
            < Math.abs(curPose.getY() - neutralZoneTargetYs[1])
        ? neutralZoneTargetYs[0]
        : neutralZoneTargetYs[1];
  }

  public static ShotSolution solve(
      Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> robotRelativeSpeedSupplier) {

    hoodAngleRadians = Launcher.getState().getHoodAngleRads();

    ChassisSpeeds robotRelative = robotRelativeSpeedSupplier.get();
    Pose2d curPose = poseSupplier.get();

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, curPose.getRotation());

    isInsideNeutralZone =
        curPose.getX() < LinesVertical.redHubCenterX
            && curPose.getX() > LinesVertical.blueHubCenterX;

    goalXMeters = isInsideNeutralZone ? neutralZoneTargetX : hubXMeters;
    goalYMeters = isInsideNeutralZone ? findClosestNeutralZoneSide(curPose) : hubYMeters;
    goalHeightMeters = isInsideNeutralZone ? neutralZoneHeightMeters : hubHeightMeters;

    // Robot-relative turret offset (meters)

    double dxTurretRobotRelative = 0.135; // TODO: find real forward offset
    double dyTurretRobotRelative = -0.14; // TODO: find real sideways offset

    // Rotate offset into field coordinates using matrix multiplication done below

    /*
        [dxTurretFieldRelative] = [cos(theta) -sin(theta)][dxTurretRelativeToRobot]
        [dyTurretFieldRelative]   [sin(theta)  cos(theta)][dyTurretRelativeToRobot]
    */

    double thetaRobot = curPose.getRotation().getRadians();
    double dxTurretFieldRelative =
        dxTurretRobotRelative * Math.cos(thetaRobot) - dyTurretRobotRelative * Math.sin(thetaRobot);
    double dyTurretFieldRelative =
        dxTurretRobotRelative * Math.sin(thetaRobot) + dyTurretRobotRelative * Math.cos(thetaRobot);

    // Actual turret position in field coordinates

    double turretXMeters = curPose.getX() + dxTurretFieldRelative;
    double turretYMeters = curPose.getY() + dyTurretFieldRelative;

    // Derive NM calculations from turret displacement instead of robot center

    double Dx = goalXMeters - turretXMeters;
    double Dy = goalYMeters - turretYMeters;
    double Dz = goalHeightMeters - shooterHeightMeters;

    double robotVx = fieldRelativeSpeeds.vxMetersPerSecond;
    double robotVy = fieldRelativeSpeeds.vyMetersPerSecond;

    double ToF = 1.0; // Initial guess of ToF for Newton's Method
    // (TODO: create initial ToF guess based on distance to target)

    for (int i = 0; i < Constants.NEWTONS_METHOD_NUM_STEPS; i++) {
      // recalculating closer approximate value of ToF after each "step"

      double vxLaunch = (Dx / ToF) - robotVx;
      double vyLaunch = (Dy / ToF) - robotVy;

      double horizontalSpeed = Math.hypot(vxLaunch, vyLaunch);

      double vzLaunch = horizontalSpeed * Math.tan(hoodAngleRadians);

      double f = (vzLaunch * ToF - 0.5 * g * ToF * ToF) - Dz; // h = vt - 1/2at^2

      // Numerical derivative for Newton's method

      double dt = 1e-4; // "small" dt to estimate derivative
      double t2 = ToF + dt;

      double vx2 = (Dx / t2) - robotVx;
      double vy2 = (Dy / t2) - robotVy;
      double h2 = Math.hypot(vx2, vy2);
      double vz2 = h2 * Math.tan(hoodAngleRadians);

      double f2 = (vz2 * t2 - 0.5 * g * t2 * t2) - Dz;

      double fPrime = (f2 - f) / dt;

      if (Math.abs(fPrime) > 1e-6) {
        ToF -= f / fPrime;
      }

      // set min bound for t
      if (ToF < 0.1) ToF = 0.1;
    }

    // Compute final launch velocity (m/s) components

    double vxLaunchMPS = (Dx / ToF) - robotVx;
    double vyLaunchMPS = (Dy / ToF) - robotVy;

    double totalHorizontalSpeedMPS = Math.hypot(vxLaunchMPS, vyLaunchMPS);
    double vzLaunchMPS = totalHorizontalSpeedMPS * Math.tan(hoodAngleRadians);

    // Shooter wheel speed (m/s) magnitude:

    double shooterSpeedMPS = Math.hypot(totalHorizontalSpeedMPS, vzLaunchMPS);

    // Shooter wheel speed (rot / s) magnitude:

    double shooterSpeedRPS = shooterSpeedMPS * MPSToRPSConversion;

    double targetXOffsetMeters = goalXMeters - robotVx * ToF;
    double targetYOffsetMeters = goalYMeters - robotVy * ToF;
    targetPose = new Pose2d(targetXOffsetMeters, targetYOffsetMeters, new Rotation2d());

    // Compute field-relative turret angle

    Rotation2d fieldRelativeTurretAngleRot2d =
        new Rotation2d(Math.atan2(vyLaunchMPS, vxLaunchMPS));

    SmarterDashboard.putNumber("Launcher/SOTM/fieldRelativeTurretAngle", fieldRelativeTurretAngleRot2d.getDegrees());
    SmarterDashboard.putNumber("Launcher/SOTM/timeOfFlight", ToF);
    SmarterDashboard.putNumber("Launcher/SOTM/desiredShooterSpeed_RPS", shooterSpeedRPS);
    SmarterDashboard.putNumber("Launcher/SOTM/currentRotation", curPose.getRotation().getDegrees());
    Logger.recordOutput("Launcher/SOTM/targetPose", targetPose);

    return new ShotSolution(ToF, shooterSpeedRPS, fieldRelativeTurretAngleRot2d, isInsideNeutralZone);
  }
}
