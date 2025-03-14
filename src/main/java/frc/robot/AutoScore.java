// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants.CoralObjective;
import frc.robot.FieldConstants.Reef;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AutoScore {
  public static final LoggedNetworkNumber xOffset =
      new LoggedNetworkNumber("AutoScore/xOffsetInches", 19);
  public static final LoggedNetworkNumber yOffset =
      new LoggedNetworkNumber("AutoScore/yOffsetInches", -3.5);
  public static final LoggedTunableNumber minDistanceReefClearAlgae =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClearAlgae", Units.inchesToMeters(18.0));
  public static final LoggedTunableNumber minDistanceReefClear =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClear", Units.inchesToMeters(12.0));

  // Radius of regular hexagon is side length
  private static final double reefRadius = Reef.faceLength;
  private static final LoggedTunableNumber maxDistanceReefLineup =
      new LoggedTunableNumber("AutoScore/MaxDistanceReefLineup", 1.5);
  private static final LoggedTunableNumber distanceSuperstructureReady =
      new LoggedTunableNumber("AutoScore/DistanceSuperstructureReady", Units.inchesToMeters(72.0));
  private static final LoggedTunableNumber[] linearXToleranceEject = {
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L1", 0.05),
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L2", 0.15),
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L3", 0.15),
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L4", 0.02)
  };
  private static final LoggedTunableNumber[] linearYToleranceEject = {
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L1", 0.05),
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L2", 0.015),
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L3", 0.015),
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L4", 0.01)
  };
  private static final LoggedTunableNumber[] maxLinearVel = {
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L1", 3),
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L2", 3),
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L3", 3),
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L4", 3)
  };
  private static final LoggedTunableNumber[] maxAngularVel = {
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L1", 3),
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L2", 3),
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L3", 3),
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L4", 3)
  };
  private static final LoggedTunableNumber thetaToleranceEject =
      new LoggedTunableNumber("AutoScore/ThetaToleranceEject", 2.0);
  private static final LoggedTunableNumber l1AlignOffsetX =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetX", 0.5);
  private static final LoggedTunableNumber l1AlignOffsetY =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetY", 0.3);
  private static final LoggedTunableNumber l1AlignOffsetDegrees =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetDegrees", 170.0);
  private static final LoggedTunableNumber minDistanceAim =
      new LoggedTunableNumber("AutoScore/MinDistanceAim", 0.2);
  private static final LoggedTunableNumber ejectTimeSeconds =
      new LoggedTunableNumber("AutoScore/EjectTimeSeconds", 0.5);

  private AutoScore() {}

  public static Command getAutoDriveBlocking(
      Drive drive,
      Supplier<CoralObjective> coralObjective,
      Supplier<FieldConstants.ReefLevel> reefLevel) {
    Supplier<Pose2d> robot = () -> AutoScore.getRobotPose(coralObjective.get());
    DriveToPose driveToPose =
        new DriveToPose(
            drive,
            () -> {
              //                      if (reefLevel.get() ==
              // FieldConstants.ReefLevel.L1) {
              //                        return getDriveTarget(
              //                            robot.get(),
              // AllianceFlipUtil.apply(getL1Pose(objective)));
              //                      }
              Pose2d goalPose = getCoralScorePose(coralObjective.get());
              return getDriveTarget(robot.get(), AllianceFlipUtil.apply(goalPose));
              //                      return AllianceFlipUtil.apply(goalPose);
            },
            robot);
    return driveToPose.until(driveToPose::atGoal);
  }

  public static Command getAutoDrive(
      Drive drive,
      Supplier<Optional<CoralObjective>> coralObjective,
      Supplier<FieldConstants.ReefLevel> reefLevel,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega) {
    Supplier<Pose2d> robot =
        () ->
            coralObjective
                .get()
                .map(AutoScore::getRobotPose)
                .orElseGet(() -> RobotState.getInstance().getEstimatedPose());
    return new DriveToPose(
        drive,
        () ->
            coralObjective
                .get()
                .map(
                    objective -> {
                      //                      if (reefLevel.get() == FieldConstants.ReefLevel.L1) {
                      //                        return getDriveTarget(
                      //                            robot.get(),
                      // AllianceFlipUtil.apply(getL1Pose(objective)));
                      //                      }
                      Pose2d goalPose = getCoralScorePose(objective);
                      return getDriveTarget(robot.get(), AllianceFlipUtil.apply(goalPose));
                      //                      return AllianceFlipUtil.apply(goalPose);
                    })
                .orElseGet(() -> RobotState.getInstance().getEstimatedPose()),
        robot,
        () ->
            DriveCommands.getLinearVelocityFromJoysticks(
                    driverX.getAsDouble(), driverY.getAsDouble())
                .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
        () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble()));
  }

  /** Get drive target. */
  public static Pose2d getDriveTarget(Pose2d robot, Pose2d goal) {
    var offset = robot.relativeTo(goal);
    double yDistance = Math.abs(offset.getY());
    double xDistance = Math.abs(offset.getX());
    double shiftXT =
        MathUtil.clamp(
            (yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)),
            0.0,
            1.0);
    double shiftYT =
        MathUtil.clamp(yDistance <= 0.2 ? 0.0 : offset.getX() / Reef.faceLength, 0.0, 1.0);
    return goal.transformBy(
        GeomUtil.toTransform2d(
            -shiftXT * maxDistanceReefLineup.get(),
            Math.copySign(shiftYT * maxDistanceReefLineup.get() * 0.8, offset.getY())));
  }

  /** Get position of robot aligned with branch for selected objective. */
  public static Pose2d getCoralScorePose(CoralObjective coralObjective) {
    return getBranchPose(coralObjective)
        .transformBy(
            new Transform2d(Inches.of(xOffset.get()), Inches.of(yOffset.get()), Rotation2d.kZero))
        .plus(new Transform2d(0, 0, Rotation2d.k180deg));
  }

  private static Pose2d getL1Pose(CoralObjective coralObjective) {
    int face = coralObjective.branchId() / 2;
    return Reef.centerFaces[face].transformBy(
        new Transform2d(
            l1AlignOffsetX.get(),
            l1AlignOffsetY.get() * (coralObjective.branchId() % 2 == 0 ? 1.0 : -1.0),
            Rotation2d.fromDegrees(
                l1AlignOffsetDegrees.get() * (coralObjective.branchId() % 2 == 0 ? 1.0 : -1.0))));
  }

  public static boolean withinDistanceToReef(Pose2d robot, double distance) {
    final double distanceToReefCenter =
        AllianceFlipUtil.apply(robot).getTranslation().getDistance(Reef.center);
    Logger.recordOutput("AutoScore/DistanceToReefCenter", distanceToReefCenter);
    return distanceToReefCenter
        <= reefRadius
            + (TunerConstants.BackLeft.LocationX - TunerConstants.BackRight.LocationX) / 2.0
            + distance;
  }

  public static boolean outOfDistanceToReef(Pose2d robot, double distance) {
    final double distanceToReefCenter =
        AllianceFlipUtil.apply(robot).getTranslation().getDistance(Reef.center);
    Logger.recordOutput("AutoScore/DistanceToReefCenter", distanceToReefCenter);
    return distanceToReefCenter
        >= reefRadius
            + (TunerConstants.BackLeft.LocationX - TunerConstants.BackRight.LocationX) / 2.0
            + distance;
  }

  public static Pose2d getRobotPose(CoralObjective coralObjective) {
    return RobotState.getInstance()
        .getReefPose(coralObjective.branchId() / 2, getCoralScorePose(coralObjective));
  }

  public static Pose2d getBranchPose(CoralObjective objective) {
    return Reef.branchPositions2d.get(objective.branchId()).get(objective.reefLevel());
  }
}
