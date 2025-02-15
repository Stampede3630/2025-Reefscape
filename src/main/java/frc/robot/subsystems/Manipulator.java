// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(Constants.kManipulatorMotorId, Constants.kManipulatorCanBus);
  private final VelocityTorqueCurrentFOC m_velocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);

    private final TorqueCurrentFOC m_tcRequest = new TorqueCurrentFOC(0);

  private final SysIdRoutine m_sysidRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          /* This is in Amps/s, but SysId only supports "volts per second" */
          Volts.of(3).per(Second),
          /* This is in Amps, but SysId only supports "volts" */
          Volts.of(20), // TODO TBD
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdManipulator_State", state.toString())
      ),new SysIdRoutine.Mechanism(
      output -> {
        /* output is actually amps, but SysId only supports "volts" */
        m_motor.setControl(m_tcRequest.withOutput(output.in(Volts)));
        
        /* also log the requested output for SysId */
        SignalLogger.writeDouble("Manipulator_Amp", output.in(Volts));
      },null, this
    )
  );

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
  
  public Manipulator() {
    m_motor.getConfigurator().apply(new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive) // Verified 2/11 CL
            .withNeutralMode(NeutralModeValue.Brake)
        ).withSlot0(new Slot0Configs()
          .withKS(70)
          .withKV(0)
          .withKA(0)
          .withKP(10)
          .withKI(10)
        )
    );
  }

  public Command velocityCommand(DoubleSupplier _velocity) {
    return startEnd(() -> m_motor.setControl(m_velocityRequest.withVelocity(_velocity.getAsDouble())), () -> m_motor.stopMotor());
  }

  public Command dutyCycleCommand(DoubleSupplier _dutyCycle) {
    return startEnd(() -> m_motor.set(_dutyCycle.getAsDouble()), () -> m_motor.stopMotor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
