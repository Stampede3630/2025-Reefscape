// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.TimedSubsystem;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Vision extends TimedSubsystem {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final LoggedNetworkBoolean useMt1 =
      new LoggedNetworkBoolean("Vision/Use MegaTag 1", true);

  public Vision(VisionConsumer consumer, VisionIO... io) {
    super("Vision");
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " is disconnected.", AlertType.kWarning);
    }
  }

  @Override
  public void timedPeriodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();
    Map<Integer, RobotState.TxTyObservation> allTxTyObservations = new HashMap<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Add tag poses (for visualize in akit)
      List<Pose3d> tagPoses = new LinkedList<>();
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      // Loop over txTy observations
      for (var observation : inputs[cameraIndex].txTyObservations) {
        if (!allTxTyObservations.containsKey(observation.tagId())
            || observation.distance() < allTxTyObservations.get(observation.tagId()).distance()) {
          // add this observation to the set if it is a new tag, or if it is closer than the
          // previous camera
          allTxTyObservations.put(observation.tagId(), observation);
        }
      }

      // Loop over pose observations and sort poses into rejects/accept. (This is what's coming from
      // MT1/2)
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            (!useMt1.get()
                    && observation
                        .type()
                        .equals(PoseObservationType.MEGATAG_1)) // MT1 is disabled and this is MT1
                || observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
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

        RobotState.VisionObservation visionObservation =
            new RobotState.VisionObservation(
                observation.pose(),
                observation.timestamp(),
                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
        //        if (DriverStation.isDisabled()) {
        //          RobotState.getInstance().seedToVisionObservation(visionObservation);
        //        }

        // Send vision observation
        consumer.accept(visionObservation);
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/TxTyObservations",
        allTxTyObservations.values().toArray(new RobotState.TxTyObservation[0]));
    Leds.getInstance().canSeeAprilTag = !allTagPoses.isEmpty();
    allTxTyObservations.values().forEach(RobotState.getInstance()::addTxTyObservation);
  }

  public Command takeSnapshot(Supplier<String> name) {
    return runOnce(
        () -> {
          for (VisionIO x : io) {
            if (x instanceof VisionIOLimelight) {
              ((VisionIOLimelight) x).takeSnapshot(name.get());
            }
          }
        });
  }

  public Command seedPoseBeforeAuto(Pose2d ppStartingPose, Distance threshold) {
    return runOnce(
        () -> {
          VisionIO.PoseObservation bestObservation =
              new VisionIO.PoseObservation(
                  0, Pose3d.kZero, 1000, 0, 0, PoseObservationType.MEGATAG_1);
          for (VisionIOInputsAutoLogged input : inputs)
            for (var observation : input.poseObservations) {
              if (observation.type().equals(PoseObservationType.MEGATAG_1)
                  && observation.ambiguity() < bestObservation.ambiguity())
                bestObservation = observation;
            }
          if (bestObservation.tagCount() > 1
              || bestObservation
                      .pose()
                      .toPose2d()
                      .getTranslation()
                      .getDistance(ppStartingPose.getTranslation())
                  < threshold.in(
                      Meters)) { // if MT1 indicates we are less than threshold meters away from PP
            // pose OR if we have two tags, seed with vision
            RobotState.getInstance().resetPose(bestObservation.pose().toPose2d());
          } else RobotState.getInstance().resetPose(ppStartingPose);
        });
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(RobotState.VisionObservation observation);
  }
}
