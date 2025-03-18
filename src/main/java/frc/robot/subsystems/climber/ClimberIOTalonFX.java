// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX motor;
  private final CANcoder encoder;

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Double> reference;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Temperature> temp;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0);
  private final StatusSignal<Angle> absolutePosition;

  public ClimberIOTalonFX() {
    motor = new TalonFX(Constants.kClimberMotorId, Constants.kSwerveCanBus);
    encoder = new CANcoder(Constants.kClimberEncoderId, Constants.kSwerveCanBus);
    config
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        .withFeedback(new FeedbackConfigs().withFusedCANcoder(encoder))
        .withSlot0(
            new Slot0Configs() // TODO needs tuning
                .withKP(0)
                .withKS(0));

    motor.getConfigurator().apply(config);

    absolutePosition = encoder.getAbsolutePosition();
    position = motor.getPosition();
    velocity = motor.getVelocity();
    reference = motor.getClosedLoopReference();
    torqueCurrent = motor.getTorqueCurrent();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    voltage = motor.getMotorVoltage();
    temp = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(Constants.kUnimportantUpdateRate, temp);
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kSomewhatImportantUpdateRate,
        position,
        velocity,
        reference,
        torqueCurrent,
        statorCurrent,
        supplyCurrent);

    ParentDevice.optimizeBusUtilizationForAll(motor, encoder);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                reference,
                torqueCurrent,
                statorCurrent,
                supplyCurrent,
                voltage,
                temp,
                absolutePosition)
            .isOK();

    inputs.absolutePosition = absolutePosition.getValueAsDouble();
    inputs.connected = connectedDebouncer.calculate(connected);
    inputs.position = position.getValueAsDouble();
    inputs.velocity = velocity.getValueAsDouble();
    inputs.reference = reference.getValueAsDouble();
    inputs.torqueCurrent = torqueCurrent.getValueAsDouble();
    inputs.statorCurrent = statorCurrent.getValueAsDouble();
    inputs.voltage = voltage.getValueAsDouble();
    inputs.temp = temp.getValueAsDouble();
  }

  @Override
  public void runPosition(double position) {
    motor.setControl(positionRequest.withPosition(position));
  }

  @Override
  public void runVolts(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void runTorqueCurrent(double amps) {
    motor.setControl(torqueCurrentFOC.withOutput(amps));
  }
}
