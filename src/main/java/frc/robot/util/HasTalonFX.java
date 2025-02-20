// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import java.util.List;

public interface HasTalonFX {
  List<TalonFX> getTalonFXs();
}
