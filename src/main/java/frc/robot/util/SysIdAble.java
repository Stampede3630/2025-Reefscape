// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public interface SysIdAble {
  Command sysIdQuasistatic(SysIdRoutine.Direction direction);

  Command sysIdDynamic(SysIdRoutine.Direction direction);
}
