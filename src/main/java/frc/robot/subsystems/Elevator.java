// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final TalonFX m_leader = new TalonFX(Constants.kElevatorLeaderId, Constants.kSwerveCanBus);
  private final TalonFX m_follower = new TalonFX(Constants.kElevatorFollowerId, Constants.kSwerveCanBus);
  // private final CANcoder m_encoder = new CANcoder(Constants.kElevatorEncoderId, Constants.kSwerveCanBus);
  private final PositionTorqueCurrentFOC m_positionRequest = new PositionTorqueCurrentFOC(0).withSlot(0);
  private final TorqueCurrentFOC m_tcRequest = new TorqueCurrentFOC(0);
  // private final StatusSignal<Angle> m_position =  m_encoder.getPosition();

  // TODO probably only need to sysid one of the elevator motors.
  private final SysIdRoutine m_sysidRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          /* This is in Amps/s, but SysId only supports "volts per second" */
          Volts.of(3).per(Second),
          /* This is in Amps, but SysId only supports "volts" */
          Volts.of(20), // TODO TBD
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdElevator_State", state.toString())
      ),new SysIdRoutine.Mechanism(
      output -> {
        /* output is actually amps, but SysId only supports "volts" */
        m_leader.setControl(m_tcRequest.withOutput(output.in(Volts)));
        
        /* also log the requested output for SysId */
        SignalLogger.writeDouble("Leader_Amps", output.in(Volts));
      },null, this
    )
  );

  public Elevator() {
    // m_encoder.getConfigurator().apply(new CANcoderConfiguration()
    //     .withMagnetSensor(new MagnetSensorConfigs()
    //         .withSensorDirection(SensorDirectionValue.Clockwise_Positive) // TODO double check
    //     )
    // );
    // m_position.setUpdateFrequency(Constants.kNonSwerveUpdateRate);

    TalonFXConfiguration config = new TalonFXConfiguration()
    .withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake)
    )
    // .withSlot0(new Slot0Configs() // TODO Tune constants for elevator
    //       .withKS(0)
    //       .withKP(0)
    //       .withKG(0)
    // )
    .withFeedback(new FeedbackConfigs()
          .withSensorToMechanismRatio(1) // TODO find inches conversion rate (or maybe it's not linear?)
          .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
    )
    .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
          .withForwardSoftLimitEnable(false)
          .withForwardSoftLimitThreshold(60) // 60 inches to be safe for now
          .withReverseSoftLimitEnable(false)
          .withReverseSoftLimitThreshold(1)
    );
    
    m_leader.getConfigurator().apply(config);
    m_follower.getConfigurator().apply(config);

    m_follower.setControl(new Follower(Constants.kElevatorLeaderId, false));
  }

  /**
   * Runs the SysId Quasistatic test in the given direction.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysidRoutine.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysidRoutine.dynamic(direction);
  }

  // public void seedPosition(double position) {
  //   m_encoder.setPosition(position);
  // }

  public Command positionCommand(DoubleSupplier _position) {
    return runOnce(() -> {
      m_leader.setControl(m_positionRequest.withPosition(_position.getAsDouble()));
    });
  }

  public Command upCommand() {
    return startEnd(() -> {m_leader.set(0.1);}, () -> {m_leader.set(0);});
  }

  public Command downCommand() {
    return startEnd(() -> {m_leader.set(-0.1);}, () -> {m_leader.set(0);});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
