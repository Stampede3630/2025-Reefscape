// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public Elevator() {}
  private TalonFX m_lift = new TalonFX(0, Constants.kManipulatorCanBus);


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
