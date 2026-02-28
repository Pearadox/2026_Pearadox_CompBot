package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotController;
import java.util.function.Supplier;

public class VisionIOFake implements VisionIO {
  private final Supplier<Pose2d> robotPoseSupplier;

  public VisionIOFake(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = true;
    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              RobotController.getFPGATime() / 1000.,
              new Pose3d(robotPoseSupplier.get()),
              0,
              2,
              1,
              PoseObservationType.PHOTONVISION)
        };
  }
}
