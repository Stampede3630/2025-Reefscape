// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  default void updateInputs(ClimberIOInputs inputs) {}

  default void runPosition(double position) {}

  default void runVolts(double volts) {}

  default void runTorqueCurrent(double amps) {}

  default void stop() {}

  @AutoLog
  class ClimberIOInputs {
    public boolean connected = false;
    public double position = 0.0;
    public double velocity = 0.0;
    public double reference = 0.0;
    public double torqueCurrent = 0.0;
    public double voltage = 0.0;
    public double statorCurrent = 0.0;
    public double supplyCurrent = 0.0;
    public double temp = 0.0;
    public double absolutePosition = 0.0;
    public double encoderPosition = 0.0;
  }
}
