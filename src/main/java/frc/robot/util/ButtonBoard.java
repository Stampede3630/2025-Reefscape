// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class ButtonBoard extends CommandXboxController {

  private final Trigger[] buttonTriggers = new Trigger[12];
  private final LoggedNetworkBoolean[] buttons = new LoggedNetworkBoolean[12];
  private final LoggedNetworkBoolean l1;
  private final LoggedNetworkBoolean l2;
  private final LoggedNetworkBoolean l3;
  private final LoggedNetworkBoolean l4;
  private final String name;

  private HashMap<ButtonLoop, Trigger> map = new HashMap<>();

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public ButtonBoard(int port) {
    this(port, port + "");
  }

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public ButtonBoard(int port, String name) {
    super(port);
    this.name = name;
    for (int i = 0; i < 12; i++) {
      int finalI = i;
      buttons[i] = new LoggedNetworkBoolean(name + "/" + (i + 1), false);
      buttonTriggers[i] =
          new Trigger(() -> buttons[finalI].get())
              .onTrue(
                  Commands.either(
                      Commands.none(),
                      Commands.waitSeconds(0.02)
                          .andThen(
                              Commands.runOnce(() -> buttons[finalI].set(false))
                                  .ignoringDisable(true)),
                      () -> super.button(finalI + 1).getAsBoolean()));
    }
    l1 = new LoggedNetworkBoolean(name + "/L1", false);
    l2 = new LoggedNetworkBoolean(name + "/L2", false);
    l3 = new LoggedNetworkBoolean(name + "/L3", false);
    l4 = new LoggedNetworkBoolean(name + "/L4", false);
  }

  public record ButtonLoop(int button, EventLoop loop) {}

  @Override
  public Trigger button(int button, EventLoop loop) {
    if (map.containsKey(new ButtonLoop(button, loop))) {
      return map.get(new ButtonLoop(button, loop));
    } else {
      map.put(
          new ButtonLoop(button, loop),
          super.button(button, loop)
              .onTrue(Commands.runOnce(() -> buttons[button - 1].set(true)).ignoringDisable(true))
              .onFalse(Commands.runOnce(() -> buttons[button - 1].set(false)).ignoringDisable(true))
              .or(buttonTriggers[button - 1]));
    }
    return map.get(new ButtonLoop(button, loop));
  }

  public Trigger l1() {
    return super.axisGreaterThan(0, .9)
        .onTrue(Commands.runOnce(() -> l1.set(true)).ignoringDisable(true))
        .onFalse(Commands.runOnce(() -> l1.set(false)).ignoringDisable(true))
        .or(l1::get)
        .onTrue(
            Commands.either(
                Commands.none(),
                Commands.waitSeconds(0.02)
                    .andThen(Commands.runOnce(() -> l1.set(false)).ignoringDisable(true)),
                () -> super.axisGreaterThan(0, .9).getAsBoolean()));
  }

  public Trigger l2() {
    return super.axisLessThan(1, -.9)
        .onTrue(Commands.runOnce(() -> l2.set(true)).ignoringDisable(true))
        .onFalse(Commands.runOnce(() -> l2.set(false)).ignoringDisable(true))
        .or(l2::get)
        .onTrue(
            Commands.either(
                Commands.none(),
                Commands.waitSeconds(0.02)
                    .andThen(Commands.runOnce(() -> l2.set(false)).ignoringDisable(true)),
                () -> super.axisLessThan(1, -.9).getAsBoolean()));
  }

  public Trigger l3() {
    return super.axisLessThan(0, -.9)
        .onTrue(Commands.runOnce(() -> l3.set(true)).ignoringDisable(true))
        .onFalse(Commands.runOnce(() -> l3.set(false)).ignoringDisable(true))
        .or(l3::get)
        .onTrue(
            Commands.either(
                Commands.none(),
                Commands.waitSeconds(0.02)
                    .andThen(Commands.runOnce(() -> l3.set(false)).ignoringDisable(true)),
                () -> super.axisLessThan(0, -.9).getAsBoolean()));
  }

  public Trigger l4() {
    return super.axisGreaterThan(1, .9)
        .onTrue(Commands.runOnce(() -> l4.set(true)).ignoringDisable(true))
        .onFalse(Commands.runOnce(() -> l4.set(false)).ignoringDisable(true))
        .or(l4::get)
        .onTrue(
            Commands.either(
                Commands.none(),
                Commands.waitSeconds(0.02)
                    .andThen(Commands.runOnce(() -> l4.set(false)).ignoringDisable(true)),
                () -> super.axisGreaterThan(1, .9).getAsBoolean()));
  }
}
