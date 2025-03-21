// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Elevator extends SubsystemBase {
  private static final String KEY = "Elevator";
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final Alert leaderDisconnectedAlert =
      new Alert("Elevator leader motor disconnected!", Alert.AlertType.kWarning);
  private final Alert followerDisconnectedAlert =
      new Alert("Elevator follower motor disconnected!", Alert.AlertType.kWarning);

  private final LoggedNetworkBoolean coastOverride =
      new LoggedNetworkBoolean(KEY + "/CoastOverride");
  @AutoLogOutput private boolean coastModeEnabled = true;
  private final LoggedTunableNumber intakeHeight =
      new LoggedTunableNumber("Elevator/intakeHeight", 0.1);
  private double setpoint = -1;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public Command setPosition(DoubleSupplier position) {
    return runOnce(
        () -> {
          this.setpoint = position.getAsDouble();
          io.runPosition(setpoint);
        });
  }

  public Command setPositionBlocking(DoubleSupplier position, Time timeout) {
    return setPosition(position).andThen(Commands.waitUntil(this::atGoal)).withTimeout(timeout);
  }

  public Command setPositionBlocking(DoubleSupplier position, double epsilon, Time timeout) {
    return setPosition(position)
        .andThen(Commands.waitUntil(() -> atGoal(epsilon)))
        .withTimeout(timeout);
  }

  @AutoLogOutput
  public boolean atGoal() {
    return Math.abs(inputs.position - setpoint) <= 0.5;
  }

  public boolean atGoal(double epsilon) {
    return Math.abs(inputs.position - setpoint) <= epsilon;
  }

  private void setCoastMode(boolean enabled) {
    if (coastModeEnabled == enabled) return;
    coastModeEnabled = enabled;
    io.setCoastMode(coastModeEnabled);
  }

  public Command upCommand() {
    return setPosition(() -> inputs.appliedPosition + 2);
  }

  public Command downCommand() {
    return setPosition(() -> inputs.appliedPosition - 2);
  }

  public Command seedPosition(DoubleSupplier position) {
    return runOnce(() -> io.seedPosition(position.getAsDouble()));
  }

  // TODO: FIX BUG IN ELEVATOR LOGIC that depends n CAN RANGE bringing elevator down for intake
  // height
  public Command intakeHeight() {
    return setPosition(intakeHeight);
  }

  public Command intakeHeightBlocking() {
    return setPositionBlocking(intakeHeight, Seconds.of(10));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(KEY, inputs);
    leaderDisconnectedAlert.set(!inputs.leaderConnected);
    followerDisconnectedAlert.set(!inputs.followerConnected);
    if (inputs.leaderTorqueCurrent < -15 && inputs.velocity == 0) {
      seedPosition(() -> inputs.reference);
    }

    setCoastMode(coastOverride.get());
  }
}
