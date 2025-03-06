// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.util.LimelightHelpers;

import java.util.*;
import java.util.function.Supplier;
import java.util.stream.Collectors;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  /**
   * Supplies the yaw from the robot gyro, used for MegaTag 2.
   */
  private final Supplier<Rotation2d> rotationSupplier;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final String name;
  private final Transform3d cameraPose;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(Transform3d cameraPose, String name, Supplier<Rotation2d> rotationSupplier) {
    this.cameraPose = cameraPose;
    this.name = name;
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;

    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);

    // set the IMU mode
    setImuMode(0);
    new Trigger(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> setImuMode(1))).onFalse(Commands.runOnce(() -> setImuMode(0)));

  }


  /**
   * Sets the IMU mode for the LL4
   *
   * @param mode 0 - Use external IMU yaw submitted via SetRobotOrientation() for MT2 localization. The internal IMU is ignored entirely.
   *              1 - Use external IMU yaw submitted via SetRobotOrientation(), and configure the LL4 internal IMU's fused yaw to match the submitted yaw value.
   *              2 - Use internal IMU for MT2 localization.
   */
  public void setImuMode(int mode) {
    LimelightHelpers.SetIMUMode(name, mode);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update target observation
    inputs.latestTargetObservation =
          new TargetObservation(
              Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Update orientation for MegaTag 2
    LimelightHelpers.SetRobotOrientation(name, rotationSupplier.get().getDegrees(),0 ,0,0,0,0);

    // Read new pose observations from MT1
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    LimelightHelpers.PoseEstimate[] mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    for (LimelightHelpers.PoseEstimate pe : mt1) {
      if (pe == null) continue;
      tagIds.addAll(Arrays.stream(pe.rawFiducials).map(rf -> rf.id).collect(Collectors.toSet()));
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              pe.timestampSeconds,

              // 3D pose estimate
              pe.pose,

              // Ambiguity, using only the first tag because ambiguity isn't applicable for multitag
             pe.rawFiducials.length>0 ? pe.rawFiducials[0].ambiguity : 0.0,

              // Tag count
              pe.tagCount,

              // Average tag distance
              pe.avgTagDist,

              // Observation type
              PoseObservationType.MEGATAG_1));
    }

    // Read new pose observations from MT1
    LimelightHelpers.PoseEstimate[] mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    for (LimelightHelpers.PoseEstimate pe : mt2) {
      if (pe == null) continue;
      tagIds.addAll(Arrays.stream(pe.rawFiducials).map(rf -> rf.id).collect(Collectors.toSet()));
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              pe.timestampSeconds,

              // 3D pose estimate
              pe.pose,

              // Ambiguity = 0 bc already disambiguated
              0.0,

              // Tag count
              pe.tagCount,

              // Average tag distance
              pe.avgTagDist,

              // Observation type
              PoseObservationType.MEGATAG_1));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }

    // get the queue of RawFiducials that we haven't processed yet
    LimelightHelpers.RawFiducial[][] rfsQueue = LimelightHelpers.getRawFiducials(name);
    List<RobotState.TxTyObservation> txTyObservations = new LinkedList<>();
    for (LimelightHelpers.RawFiducial[] rfs : rfsQueue) {
      // For each set of RawFiducials, find valid one with the lowest ambiguity
      LimelightHelpers.RawFiducial rf = Arrays.stream(rfs)
          .filter(x -> x != null && x.id < 23 && x.id > 0 && x.ambiguity < VisionConstants.maxAmbiguity)
          .min(Comparator.comparingDouble(x -> x.ambiguity))
          .orElse(null);
      if (rf == null) continue;
      txTyObservations.add( new RobotState.TxTyObservation(rf.id,
          cameraPose,
          rf.txnc,
          rf.tync ,
          rf.distToCamera, rf.timestamp* 1.0e-6));
    }

    // Save tag observations to inputs object
    inputs.txTyObservations = txTyObservations.toArray(new RobotState.TxTyObservation[0]);
  }


}
