// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.orchestra;

import org.littletonrobotics.junction.AutoLog;

public interface OrchestraIO {
  default void loadMusic(String path) {}

  default void updateInputs(OrchestraIOInputs inputs) {}

  default void play() {}

  default void pause() {}

  default void stop() {}

  @AutoLog
  class OrchestraIOInputs {
    public boolean connected = false;
    public State state = State.READY;

    enum State {
      READY,
      PLAYING,
      PAUSED,
      STOPPED
    }
  }
}
