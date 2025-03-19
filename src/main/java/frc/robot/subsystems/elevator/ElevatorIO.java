// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void setPIDF(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      double mmKA,
      double mmKV) {}

  default void runPosition(double positionRad) {}

  default boolean setCoastMode(boolean enabled) {
    return true;
  }

  default boolean seedPosition(double newPosition) {
    return true;
  }

  @AutoLog
  class ElevatorIOInputs {
    public boolean leaderConnected = false;
    public double position = 0.0;
    public double velocity = 0.0;
    public double reference = 0.0;
    public double leaderTorqueCurrent = 0.0;
    public double leaderVoltage = 0.0;
    public double leaderStatorCurrent = 0.0;
    public double leaderSupplyCurrent = 0.0;
    public double leaderTemp = 0.0;

    public boolean followerConnected = false;
    public double followerTorqueCurrent = 0.0;
    public double followerVoltage = 0.0;
    public double followerStatorCurrent = 0.0;
    public double followerSupplyCurrent = 0.0;
    public double followerTemp = 0.0;

    public double appliedPosition = 0.0;
  }
}
