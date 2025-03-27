// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;
import lombok.Setter;

public class TimeDifferentiation {
  @Setter private LinearFilter filter;
  @Getter private double lastInput;
  private double lastTime;
  @Getter private double lastValue;
  @Getter private double lastUnfilteredValue;

  public double calculate(double input) {
    // if this is the first time calculate is called, set lastValue and lastTime to the current
    // input and time
    if (lastTime == 0) {
      lastInput = input;
      lastTime = Timer.getFPGATimestamp();

      return 0;
    } else {
      double currentTime = Timer.getFPGATimestamp();
      double dt = currentTime - lastTime;
      double dv = input - lastInput;
      lastUnfilteredValue = dv / dt;
      lastInput = input;
      lastTime = currentTime;
    }

    // if filter is not null, use it to filter the derivative
    if (filter != null) {
      lastValue = filter.calculate(lastUnfilteredValue);
    } else {
      lastValue = lastUnfilteredValue;
    }
    return lastValue;
  }

  /**
   * Set the filter to use on the derivative and returns itself
   *
   * @param filter the filter to use
   * @return itself
   */
  public TimeDifferentiation withFilter(LinearFilter filter) {
    this.filter = filter;
    return this;
  }
}
