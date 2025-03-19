// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Utility class for logging code execution times. */
public abstract class TimedSubsystem extends SubsystemBase {
  private final String epochName;

  public TimedSubsystem(String epochName) {
    this.epochName = epochName;
  }

  /** Save the time elapsed since the last reset or record. */
  @Override
  public final void periodic() {
    double startTime = Timer.getFPGATimestamp();
    timedPeriodic();
    double now = Timer.getFPGATimestamp();
    Logger.recordOutput("PeriodicTimes/" + epochName + " ms", (now - startTime) * 1000.0);
  }

  public abstract void timedPeriodic();
}
