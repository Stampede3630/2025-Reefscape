// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import lombok.Getter;

@Getter
public class Region2D {
  private final Translation2d a;
  private final Translation2d b;

  public Region2D(Translation2d a, Translation2d b) {
    this.a = a;
    this.b = b;
  }

  public Region2D(double x1, double y1, double x2, double y2) {
    this(new Translation2d(x1, y1), new Translation2d(x2, y2));
  }

  public boolean inRegion(Translation2d toTest) {
    double minX = Math.min(a.getX(), b.getX());
    double maxX = Math.max(a.getX(), b.getX());
    double minY = Math.min(a.getY(), b.getY());
    double maxY = Math.max(a.getY(), b.getY());
    return toTest.getX() > minX
        && toTest.getX() < maxX
        && toTest.getY() > minY
        && toTest.getY() < maxY;
  }
}
