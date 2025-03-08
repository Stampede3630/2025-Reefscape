// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final CANBus kSwerveCanBus = new CANBus("Swerve");
  public static final CANBus kManipulatorCanBus = new CANBus("rio");
  public static final int kClimberMotorId = 20;
  public static final int kElevatorLeaderId = 17;
  public static final int kElevatorFollowerId = 18;
  public static final int kManipulatorMotorId = 1;
  public static final int kManipulatorSensorId = 19;
  public static final int kFunnelSensorId = 20;
  public static final Frequency kUnimportantUpdateRate = Hertz.of(50);

  public static final AudioConfigs kAudioConfigs =
      new AudioConfigs().withBeepOnConfig(true).withBeepOnBoot(true).withAllowMusicDurDisable(true);
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = true;
  public static double kSomewhatImportantUpdateRate = 100;
  public static double kImportantUpdateRate = 250;
  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
