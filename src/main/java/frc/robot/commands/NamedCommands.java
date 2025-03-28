// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoScore;
import frc.robot.FieldConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.vision.Vision;
import java.util.HashMap;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class NamedCommands {
  @Getter private final HashMap<String, Command> commands = new HashMap<>();
  private final Drive drive;
  private final Climber climber;
  private final Elevator elevator;
  private final Manipulator manipulator;
  private final Vision vision;

  public NamedCommands(
      Drive drive, Climber climber, Elevator elevator, Manipulator manipulator, Vision vision) {
    this.drive = drive;
    this.climber = climber;
    this.elevator = elevator;
    this.manipulator = manipulator;
    this.vision = vision;
    /*  for (int i = 0; i < 12; i++) {
        commands.put(
            "scoreCoral" + i + "L4",
            getAutoScore(new FieldConstants.CoralObjective(i, FieldConstants.ReefLevel.L4)));
      }
      commands.put("intakeCoral", elevator.intakeHeight().andThen(manipulator.autoIntake()));

      com.pathplanner.lib.auto.NamedCommands.registerCommands(commands);
    }
    */
    // Commands with ascii values! :)
    for (int i = 65; i <= 76; i++) {
      int coralPlacement = 14 - (i / 65 + i % 65);
      if (coralPlacement > 11) {
        coralPlacement -= 12;
      }
      commands.put(
          "scoreCoral" + (char) i + "L4",
          getAutoScore(
              new FieldConstants.CoralObjective(coralPlacement, FieldConstants.ReefLevel.L4)));
    }
    commands.put("intakeCoral", manipulator.autoIntake());
    commands.put("waitToGetCoral", Commands.waitUntil(manipulator.manipulatorTof()));
    com.pathplanner.lib.auto.NamedCommands.registerCommands(commands);
  }

  // private Command getAutoScore(FieldConstants.CoralObjective objective) {
  //   DoubleSupplier elevHeight = () -> objective.reefLevel().height;
  //   return elevator
  //       .setPosition(elevHeight)
  //       .alongWith(
  //           AutoScore.getAutoDriveBlocking(drive, () -> objective, () -> objective.reefLevel()))
  //       .andThen(elevator.setPositionBlocking(elevHeight, Seconds.of(10000)))
  //       .andThen(manipulator.outtake(() -> 10))
  //       .andThen(elevator.intakeHeightBlocking());
  // }

 /* private Command getAutoScore(FieldConstants.CoralObjective objective) {
    DoubleSupplier elevHeight = () -> objective.reefLevel().height;
    return Commands.waitSeconds(0.3)
        .andThen(elevator.setPosition(elevHeight))
        .alongWith(
            AutoScore.getAutoDriveBlocking(drive, () -> objective, () -> objective.reefLevel()))
        .andThen(elevator.setPositionBlocking(elevHeight, Seconds.of(1.5)))
        .andThen(Commands.waitSeconds(0.5))
        .andThen(manipulator.outtake(() -> 10))
        .andThen(elevator.intakeHeightBlocking());
  }
  */
   private Command getAutoScore(FieldConstants.CoralObjective objective) {
     DoubleSupplier elevHeight = () -> objective.reefLevel().height;
     return Commands.sequence(
         AutoScore.getAutoDriveBlocking(drive, () -> objective, () -> objective.reefLevel()),
         Commands.parallel(
            elevator.setPositionBlocking(elevHeight, Seconds.of(1.5)),
            AutoScore.getAutoDriveBlocking(drive, () -> objective, () -> objective.reefLevel())),
         Commands.waitSeconds(0.3),
         manipulator.outtake(() -> 10),
         elevator.intakeHeightBlocking());
   }
}
