// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.orchestra;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.ArrayList;
import java.util.List;

public class OrchestraIOTalonFX implements OrchestraIO {
  private final Orchestra orchestra;
  private final List<TalonFX> talons = new ArrayList<>();
  private OrchestraIOInputs.State state = OrchestraIOInputs.State.READY;

  public OrchestraIOTalonFX(List<TalonFX>... motors) {
    orchestra = new Orchestra();
    for (int i = 0; i < motors.length; i++) {
      int finalI = i;
      motors[i].forEach(
          m -> {
            orchestra.addInstrument(m, finalI);
            talons.add(m);
          });
    }
  }

  @Override
  public void loadMusic(String path) {
    state = OrchestraIOInputs.State.READY;
    orchestra.loadMusic(path);
  }

  public void play() {
    state = OrchestraIOInputs.State.PLAYING;
    orchestra.play();
  }

  public void pause() {
    state = OrchestraIOInputs.State.PAUSED;
    orchestra.pause();
  }

  public void stop() {
    state = OrchestraIOInputs.State.STOPPED;
    orchestra.stop();
  }

  @Override
  public void updateInputs(OrchestraIOInputs inputs) {
    inputs.connected = talons.stream().allMatch(TalonFX::isConnected);
    inputs.state = state;
  }
}
