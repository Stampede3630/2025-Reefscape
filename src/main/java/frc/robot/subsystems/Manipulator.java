// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(Constants.kManipulatorMotorId, Constants.kManipulatorCanBus);
  private final VelocityTorqueCurrentFOC m_request = new VelocityTorqueCurrentFOC(0);

  public Manipulator() {
    m_motor.getConfigurator().apply(new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) // TODO TBD
            .withNeutralMode(NeutralModeValue.Brake)
        )
    );
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
