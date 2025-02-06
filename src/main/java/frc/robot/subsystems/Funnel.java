// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Funnel extends SubsystemBase {
  private final TalonFX m_funnel = new TalonFX(Constants.kFunnelMotorId, Constants.kSwerveCanBus);
  private final CANrange m_canRange = new CANrange(Constants.kCanRangeId, Constants.kSwerveCanBus);
  private final StatusSignal<Boolean> m_isCoralDetected = m_canRange.getIsDetected();
  private final VelocityTorqueCurrentFOC m_funnelRequest = new VelocityTorqueCurrentFOC(0);

  public Funnel() {
    m_funnel.getConfigurator().apply(new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) // TODO TBD
            .withNeutralMode(NeutralModeValue.Brake)
        ).withSlot0(new Slot0Configs()
            .withKS(0) // TODO Tune
            .withKP(0)
        ).withFeedback(new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        )
    );


    m_canRange.getConfigurator().apply(new CANrangeConfiguration());
    m_isCoralDetected.setUpdateFrequency(Constants.kNonSwerveUpdateRate);
  }

  public boolean isDetected() {
    return m_isCoralDetected.getValue();
  }

  public Command intake() {
    return startEnd(() -> m_funnel.setControl(m_funnelRequest.withVelocity(1)),() -> m_funnel.setControl(m_funnelRequest.withVelocity(0)));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
