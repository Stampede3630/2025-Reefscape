// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotState;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber rawfiducialsSubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;
  private final int id;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(int id, String name, Supplier<Rotation2d> rotationSupplier) {
    this.id = id;
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();

    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    rawfiducialsSubscriber = table.getDoubleArrayTopic("rawfiducials").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update target observation
    TimestampedDoubleArray[] rawfiducials = rawfiducialsSubscriber.readQueue();
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    List<RobotState.TxTyObservation> txTyObservations = new LinkedList<>();
    for (var rawSample : rawfiducials) {
      if (rawSample.value.length == 0) continue;
      txTyObservations.add(
          new RobotState.TxTyObservation(
              (int) rawSample.value[0],
              id,
              Math.toRadians(rawSample.value[1]),
              Math.toRadians(rawSample.value[2]),
              rawSample.value[5],
              rawSample.timestamp));
    }

    // Save tag observations to inputs object
    inputs.txTyObservations = new RobotState.TxTyObservation[txTyObservations.size()];
    for (int i = 0; i < txTyObservations.size(); i++) {
      inputs.txTyObservations[i] = txTyObservations.get(i);
    }

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, using only the first tag because ambiguity isn't applicable for multitag
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_1));
      //      var supposedlyRobot =
      //          new Pose3d(
      //                  new Translation3d(4.073905999999999, 4.745482, 0.308102),
      //                  new Rotation3d(new Quaternion(0.5, 0, 0, 0.8660254037844386)))
      //              .plus(new Transform3d(0.46355, 0, -0.308102, new Rotation3d(0, 0, Math.PI)));
      //      Logger.recordOutput("camto19",
      // parsePose(rawSample.value).relativeTo(supposedlyRobot));
      //      Logger.recordOutput("supposedlyRObot", supposedlyRobot);
      //
      //      Logger.recordOutput(
      //          "supposedly Tag",
      //          new Pose3d(
      //              new Translation3d(4.073905999999999, 4.745482, 0.308102),
      //              new Rotation3d(new Quaternion(0.5, 0, 0, 0.8660254037844386))));
    }
    //    for (var rawSample : megatag2Subscriber.readQueue()) {
    //      if (rawSample.value.length == 0) continue;
    //      for (int i = 11; i < rawSample.value.length; i += 7) {
    //        tagIds.add((int) rawSample.value[i]);
    //      }
    //      poseObservations.add(
    //          new PoseObservation(
    //              // Timestamp, based on server timestamp of publish and latency
    //              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
    //
    //              // 3D pose estimate
    //              parsePose(rawSample.value),
    //
    //              // Ambiguity, zeroed because the pose is already disambiguated
    //              0.0,
    //
    //              // Tag count
    //              (int) rawSample.value[7],
    //
    //              // Average tag distance
    //              rawSample.value[9],
    //
    //              // Observation type
    //              PoseObservationType.MEGATAG_2));
    //    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
