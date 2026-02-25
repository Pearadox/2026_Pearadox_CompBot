package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private Supplier<ChassisSpeeds> speedsSupplier;

  public Turret(TurretIO io, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    this.io = io;
    this.speedsSupplier = chassisSpeedsSupplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public void followTarget(Supplier<Rotation2d> robotCentricAngleSupplier) {
    double setpointTurretRads = wrap(robotCentricAngleSupplier.get().getRadians());
    double setpointMotorRots = setpointTurretRads / TurretConstants.TURRET_P_COEFFICIENT;

    double ffVolts = getFF(setpointTurretRads);

    io.runPosition(setpointMotorRots, ffVolts);

    Logger.recordOutput(
        "Turret/Setpoint Turret Degrees", Units.radiansToDegrees(setpointTurretRads));
    Logger.recordOutput("Turret/Setpoint Motor Rots", setpointMotorRots);
    Logger.recordOutput("Turret/FF Volts", ffVolts);
  }

  @AutoLogOutput
  public double getTurretAngleRads() {
    return inputs.turretData.position() * TurretConstants.TURRET_P_COEFFICIENT;
  }

  @AutoLogOutput
  public double getTurretAngleDegs() {
    return Units.radiansToDegrees(getTurretAngleRads());
  }

  private double wrap(double target) {
    target = Math.IEEEremainder(target, 2 * Math.PI);

    double current = getTurretAngleRads();
    double[] candidates = new double[] {target - 2 * Math.PI, target, target + 2 * Math.PI};

    double bestAngle = target;
    double bestDist = Double.POSITIVE_INFINITY;

    for (double c : candidates) {
      if (c > TurretConstants.TURRET_SAFE_MIN && c < TurretConstants.TURRET_SAFE_MAX) {
        double dist = Math.abs(current - c);
        if (dist < bestDist) {
          bestAngle = c;
          bestDist = dist;
        }
      }
    }

    return bestAngle;
  }

  private double getFF(double setpointRads) {
    double chassisAngularVelocity = speedsSupplier.get().omegaRadiansPerSecond;

    boolean shouldApplyFF =
        Math.abs(
                    Rotation2d.fromRadians(setpointRads)
                        .minus(Rotation2d.fromRadians(getTurretAngleRads()))
                        .getRadians())
                < TurretConstants.FF_ERROR_THRESHOLD
            && Math.abs(chassisAngularVelocity) < TurretConstants.FF_CHASSIS_ROT_VELOCITY_LIMIT;

    return shouldApplyFF ? chassisAngularVelocity * TurretConstants.K_OMEGA : 0;
  }
}
