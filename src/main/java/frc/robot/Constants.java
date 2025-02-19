// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Frequency;

public class Constants {
  public static final CANBus kSwerveCanBus = new CANBus("Swerve");
  public static final CANBus kManipulatorCanBus = new CANBus("rio");

  public static final int kFunnelMotorId = 0;
  public static final int kClimberMotorId = 0;

  public static final int kElevatorLeaderId = 17;
  public static final int kElevatorFollowerId = 18;
  public static final int kManipulatorMotorId = 1;
  public static final int kCanRangeId = 0;

  public static final Frequency kNonSwerveUpdateRate = Hertz.of(250);
}
