// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final TalonFX m_leader = new TalonFX(Constants.kElevatorLeaderId, Constants.kSwerveCanBus);
  private final TalonFX m_follower = new TalonFX(Constants.kElevatorFollowerId, Constants.kSwerveCanBus);
  // private final CANcoder m_encoder = new CANcoder(Constants.kElevatorEncoderId, Constants.kSwerveCanBus);
  private double positionSetpoint = 0;
  private final MotionMagicExpoVoltage m_positionRequest = new MotionMagicExpoVoltage(positionSetpoint).withSlot(0).withEnableFOC(true);


  public Elevator() {

    TalonFXConfiguration config = new TalonFXConfiguration()
    .withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake)
    )
    .withSlot0(new Slot0Configs() // TODO Tune constants for elevator
          .withKP(1)
          .withKG(0.21875)
          .withKS(0)
          .withKV(.19)
          .withKA(0)
          .withKI(0.005)
          .withGravityType(GravityTypeValue.Elevator_Static)
    )
    .withFeedback(new FeedbackConfigs()
          .withSensorToMechanismRatio(1.09435) // TODO find inches conversion rate (or maybe it's not linear?)
          .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
    )
    .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
          .withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(60) // 60 inches to be safe for now
          .withReverseSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(0)
    )
    .withMotionMagic(new MotionMagicConfigs()
      .withMotionMagicAcceleration(0)
      .withMotionMagicCruiseVelocity(0)
      .withMotionMagicExpo_kA(.1)
      .withMotionMagicExpo_kV(.1)
    );
    
    m_leader.getConfigurator().apply(config);
    m_follower.getConfigurator().apply(config);

    m_leader.setPosition(0);
    m_follower.setPosition(0);

    m_follower.setControl(new Follower(Constants.kElevatorLeaderId, false));
    
    SmartDashboard.putBoolean("Enable Coast?", true);
  }

  public Command seedPosition(double position) {
    return runOnce(() -> {m_leader.setPosition(position);
      m_follower.setPosition(position);});
    
  }

  public Command positionCommand(DoubleSupplier _position) {
    return runOnce(() -> {
      positionSetpoint=_position.getAsDouble();
      m_leader.setControl(m_positionRequest.withPosition(positionSetpoint));
    });
  }

  public Command upCommand() {
    return positionCommand(() -> positionSetpoint+2);
  }

  public Command downCommand() {
    return positionCommand(() -> positionSetpoint-2);
  }

  public Command setNeutralMode(NeutralModeValue neutralModeValue) {
    return runOnce(() -> {m_leader.setNeutralMode(neutralModeValue); m_follower.setNeutralMode(neutralModeValue);});
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getHeight() {
    return m_leader.getPosition().refresh().getValueAsDouble();
  }

}
