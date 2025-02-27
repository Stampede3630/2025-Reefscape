// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.util.HasTalonFX;
import java.util.List;

public class ManipulatorIOTalonFX implements ManipulatorIO, HasTalonFX {
  private final TalonFX motor;
  private final CANrange tof;

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Distance> tofDistance;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

  public ManipulatorIOTalonFX() {
    motor = new TalonFX(Constants.kManipulatorMotorId, Constants.kManipulatorCanBus);
    tof = new CANrange(Constants.kCanRangeId, Constants.kManipulatorCanBus);
    config
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive) // Verified 2/11 CL
                .withNeutralMode(NeutralModeValue.Brake))
        .withSlot0(new Slot0Configs().withKS(70).withKV(0).withKA(0).withKP(10).withKI(10));

    position = motor.getPosition();
    velocity = motor.getVelocity();
    torqueCurrent = motor.getTorqueCurrent();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    voltage = motor.getMotorVoltage();
    temp = motor.getDeviceTemp();
    tofDistance = tof.getDistance();

    BaseStatusSignal.setUpdateFrequencyForAll(Constants.kImportantUpdateRate, tofDistance);
    ParentDevice.optimizeBusUtilizationForAll(tof);

    BaseStatusSignal.setUpdateFrequencyForAll(Constants.kUnimportantUpdateRate, temp);
  }

  @Override
  public void runTorqueCurrent(double amps) {
    motor.setControl(torqueCurrentRequest.withOutput(amps));
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                position, velocity, torqueCurrent, statorCurrent, supplyCurrent, voltage, temp)
            .isOK();
    inputs.connected = connectedDebouncer.calculate(connected);
    inputs.position = position.getValueAsDouble();
    inputs.velocity = velocity.getValueAsDouble();
    inputs.torqueCurrent = torqueCurrent.getValueAsDouble();
    inputs.voltage = voltage.getValueAsDouble();
    inputs.statorCurrent = statorCurrent.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
    inputs.temp = temp.getValueAsDouble();
    inputs.tofDistance = tofDistance.getValueAsDouble();
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public boolean setCoastMode(boolean enabled) {
    return motor.setNeutralMode(enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake).isOK();
  }

  @Override
  public void runVelocity(double velocity) {
    motor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public List<TalonFX> getTalonFXs() {
    return List.of(motor);
  }
}
