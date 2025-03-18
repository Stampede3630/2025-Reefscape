// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Manipulator extends SubsystemBase {
  private static final String KEY = "Manipulator";
  private final ManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert =
      new Alert("Manipulator motor disconnected!", Alert.AlertType.kWarning);
  private final LoggedNetworkBoolean coastOverride =
      new LoggedNetworkBoolean(KEY + "/CoastOverride");
  @AutoLogOutput private boolean coastModeEnabled = true;

  public Manipulator(ManipulatorIO io) {
    this.io = io;
  }

  private void setCoastMode(boolean enabled) {
    if (coastModeEnabled == enabled) return;
    coastModeEnabled = enabled;
    io.setCoastMode(coastModeEnabled);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(KEY, inputs);
    motorDisconnectedAlert.set(!inputs.connected);

    setCoastMode(coastOverride.get());
  }

  public Command runTorqueCurrent(double amps) {
    return startEnd(() -> io.runTorqueCurrent(amps), io::stop);
  }

  public Command runVelocity(DoubleSupplier velocity) {
    return startEnd(() -> io.runVelocity(velocity.getAsDouble()), io::stop);
  }

  public Command outtake(DoubleSupplier velocity) {
    Debouncer debouncer = new Debouncer(0.1);
    return runVelocity(velocity)
        .until(() -> debouncer.calculate(inputs.manipulatorTofDistance > 0.1));
  }

  public Trigger funnelTof() {
    return new Trigger(() -> io.getFunnelTofDistance() < 0.1).debounce(0.1);
  }

  public Trigger manipulatorTof() {
    return new Trigger(() -> io.getManipulatorTofDistance() < 0.1).debounce(0.1);
  }

  public Command autoIntake() {
    Debouncer debouncer = new Debouncer(0.1);
    return runVelocity(() -> 10)
        .until(() -> debouncer.calculate(io.getManipulatorTofDistance() < 0.1))
        .withTimeout(Seconds.of(5));
  }

  public Trigger haveAGamePiece() {
    return funnelTof().or(manipulatorTof());
  }
}
