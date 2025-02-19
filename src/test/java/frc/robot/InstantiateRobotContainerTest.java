// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.fail;

import org.junit.jupiter.api.Test;

public class InstantiateRobotContainerTest {
  @Test
  public void createRobotContainer() {
    // Instantiate RobotContainer
    try {
      new RobotContainer();
    } catch (Exception e) {
      e.printStackTrace();
      fail("Failed to instantiate RobotContainer, see stack trace above.");
    }
  }
}
