// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.FeedbackConfigs;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class Transfer extends SubsystemBase {
//   private final TalonFX m_transfer = new TalonFX(Constants.kTransferMotorId, Constants.kSwerveCanBus);
//   private final VelocityTorqueCurrentFOC m_transferRequest = new VelocityTorqueCurrentFOC(0);
//   public Transfer() {
//     m_transfer.getConfigurator().apply(new TalonFXConfiguration()
//         .withMotorOutput(new MotorOutputConfigs()
//             .withInverted(InvertedValue.CounterClockwise_Positive) // TODO TBD
//             .withNeutralMode(NeutralModeValue.Brake)
//         ).withFeedback(new FeedbackConfigs()
//             .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
//         )
//     );

//   }

//   public Command transfer() {
//     return startEnd(() -> m_transfer.setControl(m_transferRequest.withVelocity(1)),() -> m_transfer.setControl(m_transferRequest.withVelocity(0)));
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
