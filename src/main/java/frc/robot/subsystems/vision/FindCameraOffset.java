// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class FindCameraOffset implements Vision.VisionConsumer {
  private final LoggedNetworkNumber tagId =
      new LoggedNetworkNumber("Vision/FindCameraOffset/TagId", 1);
  private final LoggedNetworkNumber robotXInTagSpace =
      new LoggedNetworkNumber("Vision/FindCameraOffset/RobotXInTagSpace", 0);
  private final LoggedNetworkNumber robotYInTagSpace =
      new LoggedNetworkNumber("Vision/FindCameraOffset/RobotYInTagSpace", 0);
  private final LoggedNetworkNumber robotZInTagSpace =
      new LoggedNetworkNumber("Vision/FindCameraOffset/RobotZInTagSpace", 0);
  private final LoggedNetworkNumber robotYawInTagSpace =
      new LoggedNetworkNumber("Vision/FindCameraOffset/RobotYawInTagSpace", 0);

  @Override
  public void accept(RobotState.VisionObservation observation) {
    Pose3d tagPoseInFieldSpace =
        FieldConstants.AprilTagLayoutType.OFFICIAL
            .getLayout()
            .getTagPose((int) tagId.get())
            .orElse(Pose3d.kZero);
    Logger.recordOutput("Vision/FindCameraOffset/TagPoseInFieldSpace", tagPoseInFieldSpace);
    Pose3d robotPoseInTagSpace =
        new Pose3d(
            robotXInTagSpace.get(),
            robotYInTagSpace.get(),
            robotZInTagSpace.get(),
            new Rotation3d(0, 0, Math.toRadians(robotYawInTagSpace.get())));
    Logger.recordOutput("Vision/FindCameraOffset/RobotPoseInTagSpace", robotPoseInTagSpace);
    Pose3d robotPoseInFieldSpace =
        tagPoseInFieldSpace.plus(
            new Transform3d(
                robotPoseInTagSpace.getTranslation(), robotPoseInTagSpace.getRotation()));
    Logger.recordOutput("Vision/FindCameraOffset/RobotPoseInFieldSpace", robotPoseInFieldSpace);
    Pose3d cameraPoseInFieldSpace = observation.visionPose();
    Logger.recordOutput("Vision/FindCameraOffset/CameraPoseInFieldSpace", cameraPoseInFieldSpace);
    Pose3d cameraPoseInRobotSpace = cameraPoseInFieldSpace.relativeTo(robotPoseInFieldSpace);
    Logger.recordOutput("Vision/FindCameraOffset/CameraPoseInRobotSpace", cameraPoseInRobotSpace);
  }
}
