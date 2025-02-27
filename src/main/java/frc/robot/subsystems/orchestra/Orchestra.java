// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.orchestra;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class Orchestra extends SubsystemBase {
  private final OrchestraIOTalonFX io;
  private final OrchestraIOInputsAutoLogged inputs = new OrchestraIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert =
      new Alert("Manipulator motor disconnected!", Alert.AlertType.kWarning);

  public Orchestra(OrchestraIOTalonFX io) {
    this.io = io;
  }

  public Command loadMusic(Supplier<String> path) {
    return runOnce(() -> io.loadMusic(path.get()));
  }

  public Command play() {
    return runOnce(io::play);
  }

  public Command pause() {
    return runOnce(io::pause);
  }

  public Command stop() {
    return runOnce(io::stop);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    motorDisconnectedAlert.set(!inputs.connected);
  }
}
