// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class ButtonBoard extends CommandXboxController {

  private String name;
  private final LoggedNetworkBoolean a = new LoggedNetworkBoolean(name + "/A");
  private final LoggedNetworkBoolean b = new LoggedNetworkBoolean(name + "/B");
  private final LoggedNetworkBoolean x = new LoggedNetworkBoolean(name + "/X");
  private final LoggedNetworkBoolean y = new LoggedNetworkBoolean(name + "/Y");
  private final LoggedNetworkBoolean leftBumper = new LoggedNetworkBoolean(name + "/LeftBumper");
  private final LoggedNetworkBoolean rightBumper = new LoggedNetworkBoolean(name + "/RightBumper");
  private final LoggedNetworkBoolean back = new LoggedNetworkBoolean(name + "/Back");
  private final LoggedNetworkBoolean start = new LoggedNetworkBoolean(name + "/Start");
  private final LoggedNetworkBoolean leftStick = new LoggedNetworkBoolean(name + "/LeftStick");
  private final LoggedNetworkBoolean rightStick = new LoggedNetworkBoolean(name + "/RightStick");

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
  }

  @Override
  public XboxController getHID() {
    return super.getHID();
  }

  @Override
  public Trigger a() {
    return super.a().or(a::get).onTrue(Commands.runOnce(() -> a.set(false)));
  }

  @Override
  public Trigger a(EventLoop loop) {
    return super.a(loop).or(a::get).onTrue(Commands.runOnce(() -> a.set(false)));
  }

  @Override
  public Trigger b() {
    return super.b().or(b::get).onTrue(Commands.runOnce(() -> b.set(false)));
  }

  @Override
  public Trigger b(EventLoop loop) {
    return super.b(loop).or(b::get).onTrue(Commands.runOnce(() -> b.set(false)));
  }

  @Override
  public Trigger x() {
    return super.x().or(x::get).onTrue(Commands.runOnce(() -> x.set(false)));
  }

  @Override
  public Trigger x(EventLoop loop) {
    return super.x(loop).or(x::get).onTrue(Commands.runOnce(() -> x.set(false)));
  }

  @Override
  public Trigger y() {
    return super.y().or(y::get).onTrue(Commands.runOnce(() -> y.set(false)));
  }

  @Override
  public Trigger y(EventLoop loop) {
    return super.y(loop).or(y::get).onTrue(Commands.runOnce(() -> y.set(false)));
  }

  @Override
  public Trigger leftBumper() {
    return super.leftBumper()
        .or(leftBumper::get)
        .onTrue(Commands.runOnce(() -> leftBumper.set(false)));
  }

  @Override
  public Trigger leftBumper(EventLoop loop) {
    return super.leftBumper(loop)
        .or(leftBumper::get)
        .onTrue(Commands.runOnce(() -> leftBumper.set(false)));
  }

  @Override
  public Trigger rightBumper() {
    return super.rightBumper()
        .or(rightBumper::get)
        .onTrue(Commands.runOnce(() -> rightBumper.set(false)));
  }

  @Override
  public Trigger rightBumper(EventLoop loop) {
    return super.rightBumper(loop)
        .or(rightBumper::get)
        .onTrue(Commands.runOnce(() -> rightBumper.set(false)));
  }

  @Override
  public Trigger back() {
    return super.back().or(back::get).onTrue(Commands.runOnce(() -> back.set(false)));
  }

  @Override
  public Trigger back(EventLoop loop) {
    return super.back(loop).or(back::get).onTrue(Commands.runOnce(() -> back.set(false)));
  }

  @Override
  public Trigger start() {
    return super.start().or(start::get).onTrue(Commands.runOnce(() -> start.set(false)));
  }

  @Override
  public Trigger start(EventLoop loop) {
    return super.start(loop).or(start::get).onTrue(Commands.runOnce(() -> start.set(false)));
  }

  @Override
  public Trigger leftStick() {
    return super.leftStick()
        .or(leftStick::get)
        .onTrue(Commands.runOnce(() -> leftStick.set(false)));
  }

  @Override
  public Trigger leftStick(EventLoop loop) {
    return super.leftStick(loop)
        .or(leftStick::get)
        .onTrue(Commands.runOnce(() -> leftStick.set(false)));
  }

  @Override
  public Trigger rightStick() {
    return super.rightStick()
        .or(rightStick::get)
        .onTrue(Commands.runOnce(() -> rightStick.set(false)));
  }

  @Override
  public Trigger rightStick(EventLoop loop) {
    return super.rightStick(loop)
        .or(rightStick::get)
        .onTrue(Commands.runOnce(() -> rightStick.set(false)));
  }

  @Override
  public Trigger leftTrigger(double threshold, EventLoop loop) {
    return super.leftTrigger(threshold, loop);
  }

  @Override
  public Trigger leftTrigger(double threshold) {
    return super.leftTrigger(threshold);
  }

  @Override
  public Trigger leftTrigger() {
    return super.leftTrigger();
  }

  @Override
  public Trigger rightTrigger(double threshold, EventLoop loop) {
    return super.rightTrigger(threshold, loop);
  }

  @Override
  public Trigger rightTrigger(double threshold) {
    return super.rightTrigger(threshold);
  }

  @Override
  public Trigger rightTrigger() {
    return super.rightTrigger();
  }

  @Override
  public double getLeftX() {
    return super.getLeftX();
  }

  @Override
  public double getRightX() {
    return super.getRightX();
  }

  @Override
  public double getLeftY() {
    return super.getLeftY();
  }

  @Override
  public double getRightY() {
    return super.getRightY();
  }

  @Override
  public double getLeftTriggerAxis() {
    return super.getLeftTriggerAxis();
  }

  @Override
  public double getRightTriggerAxis() {
    return super.getRightTriggerAxis();
  }
}
