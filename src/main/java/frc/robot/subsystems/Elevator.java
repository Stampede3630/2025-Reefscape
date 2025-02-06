// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX m_leader = new TalonFX(Constants.kElevatorMotor1Id, Constants.kSwerveCanBus);
  private TalonFX m_follower = new TalonFX(Constants.kElevatorMotor2Id, Constants.kSwerveCanBus);

  public Elevator() {

    m_leader.getConfigurator().apply(new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
    
    );
    m_follower.getConfigurator().apply(new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
    
    );
    m_follower.setControl(new Follower(Constants.kElevatorMotor1Id, false));
  }

  public Command up() {
    return startEnd(() -> {m_leader.set(0.1);}, () -> {m_leader.set(0);});
  }

  public Command down() {
    return startEnd(() -> {m_leader.set(-0.1);}, () -> {m_leader.set(0);});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
