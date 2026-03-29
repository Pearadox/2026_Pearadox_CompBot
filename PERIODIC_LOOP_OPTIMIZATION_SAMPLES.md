# Periodic Loop Optimization Samples (Findings 1-4)

This document provides specific code samples to address the top four loop-overrun findings.

## Finding 1: Reduce Per-Loop Allocations in Drive/Module Odometry

### Goal
Avoid repeated array/object creation in odometry paths that run every scheduler cycle.

### Sample A: Reuse odometry buffers in `Module`

File: `src/main/java/frc/robot/subsystems/drive/Module.java`

```java
public class Module {
  // Keep this field, but treat it as a reusable buffer
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[0];

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    int sampleCount = inputs.odometryTimestamps.length;

    // Resize only when sample count changes
    if (odometryPositions.length != sampleCount) {
      odometryPositions = new SwerveModulePosition[sampleCount];
      for (int i = 0; i < sampleCount; i++) {
        odometryPositions[i] = new SwerveModulePosition();
      }
    }

    // Update existing objects instead of creating new ones each cycle
    for (int i = 0; i < sampleCount; i++) {
      odometryPositions[i].distanceMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
      odometryPositions[i].angle = inputs.odometryTurnPositions[i];
    }

    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);

    double newTimestamp = RobotController.getFPGATime();
    double deltaHours = (newTimestamp - lastTimestamp) / 3.6e9;
    lastTimestamp = newTimestamp;

    EnergyTracker.reportCurrentUsage(deltaHours, Compeartment.DRIVE_MOTORS, inputs.driveSupplyCurrentAmps);
    EnergyTracker.reportCurrentUsage(deltaHours, Compeartment.TURN_MOTORS, inputs.turnSupplyCurrentAmps);
  }
}
```

Notes:
- If your WPILib version has immutable `SwerveModulePosition` fields, keep the resize guard and only allocate on size changes, then replace entries (`odometryPositions[i] = ...`) inside the loop.

### Sample B: Reuse scratch arrays in `Drive.periodic`

File: `src/main/java/frc/robot/subsystems/drive/Drive.java`

```java
public class Drive extends SubsystemBase {
  private final SwerveModulePosition[] modulePositionsScratch = {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };
  private final SwerveModulePosition[] moduleDeltasScratch = {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };

  @Override
  public void periodic() {
    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;

    for (int i = 0; i < sampleCount; i++) {
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        SwerveModulePosition p = modules[moduleIndex].getOdometryPositions()[i];
        modulePositionsScratch[moduleIndex].distanceMeters = p.distanceMeters;
        modulePositionsScratch[moduleIndex].angle = p.angle;

        moduleDeltasScratch[moduleIndex].distanceMeters =
            p.distanceMeters - lastModulePositions[moduleIndex].distanceMeters;
        moduleDeltasScratch[moduleIndex].angle = p.angle;

        lastModulePositions[moduleIndex].distanceMeters = p.distanceMeters;
        lastModulePositions[moduleIndex].angle = p.angle;
      }

      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltasScratch);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositionsScratch);
    }

    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }
}
```

---

## Finding 2: Reduce Vision Periodic Allocation/Logging Cost

### Goal
Lower GC pressure and logging bandwidth from `Vision.periodic`.

### Sample: Switch to `ArrayList`, cache field dimensions, and throttle heavy logs

File: `src/main/java/frc/robot/subsystems/vision/Vision.java`

```java
import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase {
  private static final int HEAVY_LOG_DIVISOR = 5; // 50 Hz loop -> 10 Hz heavy logs
  private int heavyLogCounter = 0;

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    final double fieldLength = aprilTagLayout.getFieldLength();
    final double fieldWidth = aprilTagLayout.getFieldWidth();
    final boolean shouldLogHeavy = (++heavyLogCounter % HEAVY_LOG_DIVISOR) == 0;

    List<Pose3d> allTagPoses = shouldLogHeavy ? new ArrayList<>(32) : null;
    List<Pose3d> allRobotPoses = shouldLogHeavy ? new ArrayList<>(32) : null;
    List<Pose3d> allRobotPosesAccepted = shouldLogHeavy ? new ArrayList<>(32) : null;
    List<Pose3d> allRobotPosesRejected = shouldLogHeavy ? new ArrayList<>(32) : null;

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      List<Pose3d> tagPoses = shouldLogHeavy ? new ArrayList<>(inputs[cameraIndex].tagIds.length) : null;
      List<Pose3d> robotPoses = shouldLogHeavy ? new ArrayList<>(inputs[cameraIndex].poseObservations.length) : null;
      List<Pose3d> robotPosesAccepted = shouldLogHeavy ? new ArrayList<>(inputs[cameraIndex].poseObservations.length) : null;
      List<Pose3d> robotPosesRejected = shouldLogHeavy ? new ArrayList<>(inputs[cameraIndex].poseObservations.length) : null;

      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (shouldLogHeavy && tagPose.isPresent()) tagPoses.add(tagPose.get());
      }

      for (var observation : inputs[cameraIndex].poseObservations) {
        Pose3d pose = observation.pose();
        boolean rejectPose =
            observation.tagCount() == 0
                || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
                || Math.abs(pose.getZ()) > maxZError
                || pose.getX() < 0.0
                || pose.getX() > fieldLength
                || pose.getY() < 0.0
                || pose.getY() > fieldWidth;

        if (shouldLogHeavy) {
          robotPoses.add(pose);
          if (rejectPose) robotPosesRejected.add(pose);
          else robotPosesAccepted.add(pose);
        }

        if (rejectPose) continue;

        double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        consumer.accept(pose.toPose2d(), observation.timestamp(), VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      if (shouldLogHeavy) {
        Logger.recordOutput("Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPosesAccepted", robotPosesAccepted.toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPosesRejected", robotPosesRejected.toArray(Pose3d[]::new));
        allTagPoses.addAll(tagPoses);
        allRobotPoses.addAll(robotPoses);
        allRobotPosesAccepted.addAll(robotPosesAccepted);
        allRobotPosesRejected.addAll(robotPosesRejected);
      }
    }

    if (shouldLogHeavy) {
      Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(Pose3d[]::new));
      Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(Pose3d[]::new));
      Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(Pose3d[]::new));
      Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(Pose3d[]::new));
    }
  }
}
```

---

## Finding 3: Move Non-Critical Visualization/Energy Logging Off Every Cycle

### Goal
Keep the main scheduler path focused on controls and sensors.

### Sample: Run heavy telemetry every N loops

File: `src/main/java/frc/robot/Robot.java`

```java
public class Robot extends LoggedRobot {
  private static final int SLOW_TELEMETRY_DIVISOR = 5; // 50 Hz loop -> 10 Hz
  private int slowTelemetryCounter = 0;

  @Override
  public void robotPeriodic() {
    LoggedTracer.reset();
    PhoenixUtil.refreshAll();
    LoggedTracer.record("PhoenixRefresh");

    CommandScheduler.getInstance().run();
    LoggedTracer.record("CommandScheduler");

    // Non-critical telemetry and model updates at reduced frequency
    if (++slowTelemetryCounter >= SLOW_TELEMETRY_DIVISOR) {
      slowTelemetryCounter = 0;
      robotContainer.visualizer.periodic();
      EnergyTracker.periodic();
      LoggedTracer.record("SlowTelemetry");
    }

    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    alliance = allianceOptional.orElse(Alliance.Blue);
    isHubCurrentlyActive = isHubActive();
    Logger.recordOutput("Robot/isHubActive", isHubCurrentlyActive);
  }
}
```

Optional extra step:
- Gate visualizer updates behind a debug flag (for example `Constants.ENABLE_VISUALIZER_LOGS`).

---

## Finding 4: Reduce Synchronous Phoenix Refresh Cost

### Goal
Avoid spending the same refresh budget in all robot states and for all signal groups.

### Sample A: Decimate refresh in disabled mode

File: `src/main/java/frc/robot/Robot.java`

```java
public class Robot extends LoggedRobot {
  private int disabledRefreshCounter = 0;
  private static final int DISABLED_REFRESH_DIVISOR = 5;

  @Override
  public void robotPeriodic() {
    LoggedTracer.reset();

    boolean doRefresh =
        DriverStation.isEnabled() || (++disabledRefreshCounter % DISABLED_REFRESH_DIVISOR) == 0;
    if (doRefresh) {
      PhoenixUtil.refreshAll();
    }

    LoggedTracer.record("PhoenixRefresh");
    CommandScheduler.getInstance().run();
    // ...
  }
}
```

### Sample B: Split fast vs slow Phoenix signal groups

File: `src/main/java/frc/robot/util/PhoenixUtil.java`

```java
public final class PhoenixUtil {
  private static BaseStatusSignal[] fastCanivoreSignals = new BaseStatusSignal[0];
  private static BaseStatusSignal[] slowCanivoreSignals = new BaseStatusSignal[0];
  private static BaseStatusSignal[] fastRioSignals = new BaseStatusSignal[0];
  private static BaseStatusSignal[] slowRioSignals = new BaseStatusSignal[0];

  public static void registerFastSignals(boolean canivore, BaseStatusSignal... signals) {
    if (canivore) fastCanivoreSignals = append(fastCanivoreSignals, signals);
    else fastRioSignals = append(fastRioSignals, signals);
  }

  public static void registerSlowSignals(boolean canivore, BaseStatusSignal... signals) {
    if (canivore) slowCanivoreSignals = append(slowCanivoreSignals, signals);
    else slowRioSignals = append(slowRioSignals, signals);
  }

  public static void refreshFast() {
    if (fastCanivoreSignals.length > 0) BaseStatusSignal.refreshAll(fastCanivoreSignals);
    if (fastRioSignals.length > 0) BaseStatusSignal.refreshAll(fastRioSignals);
  }

  public static void refreshSlow() {
    if (slowCanivoreSignals.length > 0) BaseStatusSignal.refreshAll(slowCanivoreSignals);
    if (slowRioSignals.length > 0) BaseStatusSignal.refreshAll(slowRioSignals);
  }

  private static BaseStatusSignal[] append(BaseStatusSignal[] base, BaseStatusSignal[] add) {
    BaseStatusSignal[] out = new BaseStatusSignal[base.length + add.length];
    System.arraycopy(base, 0, out, 0, base.length);
    System.arraycopy(add, 0, out, base.length, add.length);
    return out;
  }
}
```

Then in `Robot.robotPeriodic()`:

```java
PhoenixUtil.refreshFast();
if ((loopCounter++ % 5) == 0) {
  PhoenixUtil.refreshSlow();
}
```

---

## Profiling the Periodic Loop with `LoggedTracer`

The codebase already includes `LoggedTracer` (`src/main/java/frc/robot/util/LoggedTracer.java`), which records elapsed milliseconds since the last reset/record call as a keyed log entry. Placing checkpoints around every major block of `robotPeriodic` gives you a breakdown of where time actually goes, viewable in AdvantageScope under `LoggedTracer/`.

### Sample: Full instrumented `robotPeriodic`

File: `src/main/java/frc/robot/Robot.java`

```java
@Override
public void robotPeriodic() {
  LoggedTracer.reset();                              // start the stopwatch

  PhoenixUtil.refreshAll();
  LoggedTracer.record("PhoenixRefresh");             // time: CAN signal refresh

  CommandScheduler.getInstance().run();              // runs all subsystem periodic() methods
  LoggedTracer.record("CommandScheduler");           // time: entire subsystem tree

  robotContainer.visualizer.periodic();
  LoggedTracer.record("RobotVisualizer");            // time: 3D pose visualization

  EnergyTracker.periodic();
  LoggedTracer.record("EnergyTracker");              // time: energy logging

  Optional<Alliance> allianceOptional = DriverStation.getAlliance();
  alliance = allianceOptional.orElse(Alliance.Blue);
  isHubCurrentlyActive = isHubActive();
  Logger.recordOutput("Robot/isHubActive", isHubCurrentlyActive);
  LoggedTracer.record("Misc");                       // time: alliance + hub check
}
```

This emits keys like `LoggedTracer/PhoenixRefreshMS`, `LoggedTracer/CommandSchedulerMS`, etc., every loop cycle. In AdvantageScope, plot all of them simultaneously to see which block owns the overrun.

---

### Sample: Instrument individual subsystem periodics to narrow inside `CommandScheduler`

Since `CommandScheduler.getInstance().run()` calls every registered subsystem's `periodic()` in registration order, you need per-subsystem traces inside each class. Use the same `LoggedTracer` calls in whichever subsystem you suspect.

File: `src/main/java/frc/robot/subsystems/drive/Drive.java`

```java
@Override
public void periodic() {
  odometryLock.lock();
  gyroIO.updateInputs(gyroInputs);
  Logger.processInputs("Drive/Gyro", gyroInputs);
  for (var module : modules) {
    module.periodic();
  }
  odometryLock.unlock();
  Logger.recordOutput("Drive/Timing/InputsMs",
      (Timer.getFPGATimestamp() - _dbgStart) * 1000.0); // see note below

  // ... rest of periodic
}
```

Alternatively, add a self-timer pattern inside any subsystem without touching `LoggedTracer`:

```java
@Override
public void periodic() {
  double t0 = Timer.getFPGATimestamp();

  // ... all periodic work ...

  Logger.recordOutput("Drive/Timing/TotalMs", (Timer.getFPGATimestamp() - t0) * 1000.0);
}
```

You can do the same in `Vision`, `Turret`, `Launcher`, etc. without them interfering with each other because each uses its own local `t0`.

---

### Sample: Detect worst-case loop cycles (max tracking)

Log a rolling maximum so the worst overrun cycle stands out even after the event.

```java
// Add these fields to Robot.java
private double maxLoopMs = 0.0;
private static final double LOOP_WARN_MS = 18.0;

@Override
public void robotPeriodic() {
  double loopStart = Timer.getFPGATimestamp();
  LoggedTracer.reset();

  PhoenixUtil.refreshAll();
  LoggedTracer.record("PhoenixRefresh");

  CommandScheduler.getInstance().run();
  LoggedTracer.record("CommandScheduler");

  robotContainer.visualizer.periodic();
  EnergyTracker.periodic();
  LoggedTracer.record("SlowTelemetry");

  Optional<Alliance> allianceOptional = DriverStation.getAlliance();
  alliance = allianceOptional.orElse(Alliance.Blue);
  isHubCurrentlyActive = isHubActive();
  Logger.recordOutput("Robot/isHubActive", isHubCurrentlyActive);

  double loopMs = (Timer.getFPGATimestamp() - loopStart) * 1000.0;
  if (loopMs > maxLoopMs) maxLoopMs = loopMs;
  Logger.recordOutput("Robot/Timing/LoopMs", loopMs);
  Logger.recordOutput("Robot/Timing/MaxLoopMs", maxLoopMs);
  if (loopMs > LOOP_WARN_MS) {
    Logger.recordOutput("Robot/Timing/OverrunMs", loopMs);
  }
}
```

`Robot/Timing/LoopMs` gives you a cycle-by-cycle trace; `Robot/Timing/MaxLoopMs` captures the worst-ever value so it persists in the log even if you miss the moment. `Robot/Timing/OverrunMs` only writes when you are actually close to the deadline, making overruns easy to filter for in AdvantageScope.

---

### Reading the results in AdvantageScope

1. Open the `.wpilog` file.
2. Graph `LoggedTracer/PhoenixRefreshMS`, `LoggedTracer/CommandSchedulerMS`, and any subsystem-level keys together in a line chart.
3. Sort by max value to find the single largest contributor.
4. For `CommandScheduler`, add per-subsystem `Logger.recordOutput("X/Timing/Ms", ...)` only in the subsystem that ranked highest, then re-run to narrow further.
5. The `Robot/Timing/OverrunMs` key can be used as a filter: zoom the timeline to any timestamp where it has a value to see exactly what was running that cycle.

---

## Quick Validation Checklist

1. Add `LoggedTracer.record(...)` around each major block and compare before/after averages and max values.
2. Watch garbage collection pauses in driver station logs after reducing allocations.
3. Confirm no control quality regressions (pose drift, shot accuracy, turret tracking) after decimation.
4. Keep safety-critical control, odometry, and command scheduling at full rate; decimate only telemetry/diagnostic work.
