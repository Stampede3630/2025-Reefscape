// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TimedSubsystem;
import java.util.Objects;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Elevator extends TimedSubsystem {
  private static final String KEY = "Elevator";
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final Alert leaderDisconnectedAlert =
      new Alert("Elevator leader motor disconnected!", Alert.AlertType.kWarning);
  private final Alert followerDisconnectedAlert =
      new Alert("Elevator follower motor disconnected!", Alert.AlertType.kWarning);

  private final LoggedNetworkBoolean coastOverride =
      new LoggedNetworkBoolean(KEY + "/CoastOverride", false);
  private final LoggedTunableNumber intakeHeight =
      new LoggedTunableNumber("Elevator/intakeHeight", 0.1);
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 1);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.21875);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", .19);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0.005);
  private final LoggedTunableNumber mmKa = new LoggedTunableNumber("Elevator/mmKa", 0.075);
  private final LoggedTunableNumber mmKv = new LoggedTunableNumber("Elevator/mmKv", 0.075);

  private final LoggedTunableNumber kPDown = new LoggedTunableNumber("Elevator/kPDown", 1);
  private final LoggedTunableNumber kDDown = new LoggedTunableNumber("Elevator/kDDown", 0);
  private final LoggedTunableNumber kGDown = new LoggedTunableNumber("Elevator/kGDown", 0.21875);
  private final LoggedTunableNumber kSDown = new LoggedTunableNumber("Elevator/kSDown", 0);
  private final LoggedTunableNumber kVDown = new LoggedTunableNumber("Elevator/kVDown", .1);
  private final LoggedTunableNumber kADown = new LoggedTunableNumber("Elevator/kADown", 0);
  private final LoggedTunableNumber kIDown = new LoggedTunableNumber("Elevator/kIDown", 0.005);
  private final LoggedTunableNumber mmKaDown = new LoggedTunableNumber("Elevator/mmKaDown", 0.1);
  private final LoggedTunableNumber mmKvDown = new LoggedTunableNumber("Elevator/mmKvDown", 0.075);
  private final Debouncer elevatorResetDebouncer = new Debouncer(1);
  @AutoLogOutput private boolean coastModeEnabled = true;
  private double setpoint = -1;

  public Elevator(ElevatorIO io) {
    super("Elevator");
    this.io = io;

    io.setPIDFSlot0(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());

    io.setPIDFSlot1(
        kPDown.get(),
        kIDown.get(),
        kDDown.get(),
        kSDown.get(),
        kVDown.get(),
        kADown.get(),
        kGDown.get());

    io.setMmConstants(mmKv.get(), mmKa.get());
  }

  public Command setPosition(DoubleSupplier position) {
    return runOnce(
        () -> {
          if (inputs.position > position.getAsDouble()) { // going down
            //            System.out.println("going down. SP: " + setpoint + " pos: " +
            // position.getAsDouble());
            //            io.setMmConstants(mmKvDown.get(), mmKaDown.get());
            io.runPosition(position.getAsDouble(), 1);
          } else {
            //            System.out.println("going up. SP: " + setpoint + " pos: " +
            // position.getAsDouble());

            //            io.setMmConstants(mmKv.get(), mmKa.get());
            io.runPosition(position.getAsDouble(), 0);
          }
          this.setpoint = position.getAsDouble();
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
    return setPositionBlocking(intakeHeight, 20, Seconds.of(10));
  }

  @Override
  public void timedPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs(KEY, inputs);
    leaderDisconnectedAlert.set(!inputs.leaderConnected);
    followerDisconnectedAlert.set(!inputs.followerConnected);
    //    if (inputs.leaderTorqueCurrent < -15 && inputs.velocity == 0) {
    //      seedPosition(() -> inputs.reference);
    //    }

    if (kP.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kG.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode()))
      io.setPIDFSlot0(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());

    if (kPDown.hasChanged(hashCode())
        || kIDown.hasChanged(hashCode())
        || kDDown.hasChanged(hashCode())
        || kGDown.hasChanged(hashCode())
        || kSDown.hasChanged(hashCode())
        || kVDown.hasChanged(hashCode())
        || kADown.hasChanged(hashCode()))
      io.setPIDFSlot1(
          kPDown.get(),
          kIDown.get(),
          kDDown.get(),
          kSDown.get(),
          kVDown.get(),
          kADown.get(),
          kGDown.get());
    if (mmKa.hasChanged(hashCode()) || mmKv.hasChanged(hashCode()))
      io.setMmConstants(mmKv.get(), mmKa.get());

    if (elevatorResetDebouncer.calculate(
        inputs.position < 2 && Math.abs(inputs.leaderTorqueCurrent) > 20)) // stalling at the bottom
    io.seedPosition(0);
    setCoastMode(coastOverride.get());
  }

  @Override
  public boolean equals(Object o) {
    if (!(o instanceof Elevator elevator)) return false;
    return coastModeEnabled == elevator.coastModeEnabled
        && Double.compare(setpoint, elevator.setpoint) == 0
        && Objects.equals(io, elevator.io)
        && Objects.equals(inputs, elevator.inputs)
        && Objects.equals(leaderDisconnectedAlert, elevator.leaderDisconnectedAlert)
        && Objects.equals(followerDisconnectedAlert, elevator.followerDisconnectedAlert)
        && Objects.equals(coastOverride, elevator.coastOverride)
        && Objects.equals(intakeHeight, elevator.intakeHeight)
        && Objects.equals(kP, elevator.kP)
        && Objects.equals(kD, elevator.kD)
        && Objects.equals(kG, elevator.kG)
        && Objects.equals(kS, elevator.kS)
        && Objects.equals(kV, elevator.kV)
        && Objects.equals(kA, elevator.kA)
        && Objects.equals(kI, elevator.kI)
        && Objects.equals(mmKa, elevator.mmKa)
        && Objects.equals(mmKv, elevator.mmKv);
  }

  @Override
  public int hashCode() {
    return Objects.hash(
        io,
        inputs,
        leaderDisconnectedAlert,
        followerDisconnectedAlert,
        coastOverride,
        intakeHeight,
        kP,
        kD,
        kG,
        kS,
        kV,
        kA,
        kI,
        mmKa,
        mmKv,
        coastModeEnabled,
        setpoint);
  }
}
