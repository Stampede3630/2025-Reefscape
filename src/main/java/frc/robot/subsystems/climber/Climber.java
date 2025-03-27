// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.TimedSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends TimedSubsystem {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    super("Climber");
    this.io = io;
  }

  @Override
  public void timedPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public Command runTorqueCurrent(DoubleSupplier tc) {
    return startEnd(() -> io.runTorqueCurrent(tc.getAsDouble()), io::stop)
        .withName("Climber Run Torque Current");
  }

  public Command setPosition(DoubleSupplier position) {
    return runOnce(() -> io.runPosition(position.getAsDouble())).withName("Climber Set Position");
  }

  public Command runBangBang(DoubleSupplier tc, DoubleSupplier position) {
    return startEnd(() -> io.runTorqueCurrent(tc.getAsDouble()), () -> io.stop())
        .until(
            () -> {
              if (tc.getAsDouble() > 0) // running forwards
              return inputs.absolutePosition > position.getAsDouble();
              else if (tc.getAsDouble() < 0) // running backwards
              return inputs.absolutePosition < position.getAsDouble();
              return true;
            });
  }

  public Command stop() {
    return runOnce(io::stop).withName("Climber Stop");
  }
}
