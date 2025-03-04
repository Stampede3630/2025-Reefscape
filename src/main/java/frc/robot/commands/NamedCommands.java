// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.vision.Vision;
import java.util.HashMap;
import lombok.Getter;

public class NamedCommands {
  @Getter private final HashMap<String, Command> commands = new HashMap<String, Command>();
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

    commands.put("scoreCoral", elevator.setPositionBlocking(() -> 30, Seconds.of(3)).andThen());
    commands.put("intakeCoral", manipulator.autoIntake());

    com.pathplanner.lib.auto.NamedCommands.registerCommands(commands);
  }
}
