// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class ButtonBoard extends CommandXboxController {

  private final Trigger[] buttonTriggers = new Trigger[12];
  private final LoggedNetworkBoolean[] buttons = new LoggedNetworkBoolean[12];
  private final LoggedNetworkBoolean l1;
  private final LoggedNetworkBoolean l2;
  private final LoggedNetworkBoolean l3;
  private final LoggedNetworkBoolean l4;
  private final String name;

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
                  Commands.waitSeconds(0.02)
                      .andThen(Commands.runOnce(() -> buttons[finalI].set(false))));
    }
    l1 = new LoggedNetworkBoolean(name + "/L1");
    l2 = new LoggedNetworkBoolean(name + "/L2");
    l3 = new LoggedNetworkBoolean(name + "/L3");
    l4 = new LoggedNetworkBoolean(name + "/L4");
  }

  @Override
  public Trigger button(int button, EventLoop loop) {
    return super.button(button, loop)
        .onTrue(Commands.runOnce(() -> buttons[button - 1].set(true)))
        .onFalse(Commands.runOnce(() -> buttons[button - 1].set(false)))
        .or(buttonTriggers[button - 1]);
  }

  @Override
  public Trigger pov(int angle) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger pov(int pov, int angle, EventLoop loop) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger povUp() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger povUpRight() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger povRight() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger povDownRight() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger povDown() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger povDownLeft() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger povLeft() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger povUpLeft() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger povCenter() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  public Trigger l1() {
    return super.axisGreaterThan(0, .9)
        .onTrue(Commands.runOnce(() -> l1.set(true)))
        .onFalse(Commands.runOnce(() -> l1.set(false)))
        .or(l1::get)
        .onTrue(Commands.waitSeconds(0.02).andThen(Commands.runOnce(() -> l1.set(false))));
  }

  public Trigger l2() {
    return super.axisLessThan(1, -.9)
        .onTrue(Commands.runOnce(() -> l2.set(true)))
        .onFalse(Commands.runOnce(() -> l2.set(false)))
        .or(l2::get)
        .onTrue(Commands.waitSeconds(0.02).andThen(Commands.runOnce(() -> l2.set(false))));
  }

  public Trigger l3() {
    return super.axisGreaterThan(0, -.9)
        .onTrue(Commands.runOnce(() -> l3.set(true)))
        .onFalse(Commands.runOnce(() -> l3.set(false)))
        .or(l3::get)
        .onTrue(Commands.waitSeconds(0.02).andThen(Commands.runOnce(() -> l3.set(false))));
  }

  public Trigger l4() {
    return super.axisLessThan(1, .9)
        .onTrue(Commands.runOnce(() -> l4.set(true)))
        .onFalse(Commands.runOnce(() -> l4.set(false)))
        .or(l4::get)
        .onTrue(Commands.waitSeconds(0.02).andThen(Commands.runOnce(() -> l4.set(false))));
  }

  @Override
  public Trigger axisMagnitudeGreaterThan(int axis, double threshold, EventLoop loop) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger axisMagnitudeGreaterThan(int axis, double threshold) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public void setRumble(GenericHID.RumbleType type, double value) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public boolean isConnected() {
    return super.isConnected();
  }

  @Override
  public XboxController getHID() {
    return super.getHID();
  }

  @Override
  public Trigger button(int button) {
    return this.button(button, CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  @Override
  public Trigger a() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger a(EventLoop loop) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger b() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger b(EventLoop loop) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger x() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger x(EventLoop loop) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger y() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger y(EventLoop loop) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger leftBumper() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger leftBumper(EventLoop loop) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger rightBumper() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger rightBumper(EventLoop loop) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger back() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger back(EventLoop loop) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger start() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger start(EventLoop loop) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger leftStick() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger leftStick(EventLoop loop) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger rightStick() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger rightStick(EventLoop loop) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger leftTrigger(double threshold, EventLoop loop) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger leftTrigger(double threshold) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger leftTrigger() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger rightTrigger(double threshold, EventLoop loop) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger rightTrigger(double threshold) {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public Trigger rightTrigger() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public double getLeftX() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public double getRightX() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public double getLeftY() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public double getRightY() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public double getLeftTriggerAxis() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }

  @Override
  public double getRightTriggerAxis() {
    throw new UnsupportedOperationException(
        "ButtonBoard does not support this. Use button(int button) :Trigger instead");
  }
}
