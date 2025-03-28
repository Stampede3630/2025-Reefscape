// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.util.HasTalonFX;
import java.util.List;

public class ElevatorIOTalonFX implements ElevatorIO, HasTalonFX {
  private final TalonFX leader;

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Double> reference;
  private final StatusSignal<Current> leaderTorqueCurrent;
  private final StatusSignal<Current> leaderStatorCurrent;
  private final StatusSignal<Current> leaderSupplyCurrent;
  private final StatusSignal<Voltage> leaderVoltage;
  private final StatusSignal<Temperature> leaderTemp;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Current> followerStatorCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Voltage> followerVoltage;
  private final StatusSignal<Temperature> followerTemp;

  private final TalonFX follower;
  private final Debouncer leaderConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.5);
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private double positionSetpoint = 0;
  private final MotionMagicExpoVoltage positionRequest =
      new MotionMagicExpoVoltage(positionSetpoint).withSlot(0).withEnableFOC(true);

  public ElevatorIOTalonFX() {
    leader = new TalonFX(Constants.kElevatorLeaderId, Constants.kSwerveCanBus);
    follower = new TalonFX(Constants.kElevatorFollowerId, Constants.kSwerveCanBus);

    config
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(1.09435)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
        .withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(60) // 60 inches to be safe for now
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(0))
        .withAudio(Constants.kAudioConfigs);

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);

    leader.setPosition(0);
    position = leader.getPosition();
    velocity = leader.getVelocity();
    reference = leader.getClosedLoopReference();
    leaderTorqueCurrent = leader.getTorqueCurrent();
    leaderStatorCurrent = leader.getStatorCurrent();
    leaderSupplyCurrent = leader.getSupplyCurrent();
    leaderVoltage = leader.getMotorVoltage();
    leaderTemp = leader.getDeviceTemp();
    followerTorqueCurrent = follower.getTorqueCurrent();
    followerStatorCurrent = follower.getStatorCurrent();
    followerSupplyCurrent = follower.getSupplyCurrent();
    followerVoltage = follower.getMotorVoltage();
    followerTemp = follower.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kUnimportantUpdateRate, leaderTemp, followerTemp);
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kSomewhatImportantUpdateRate,
        position,
        velocity,
        reference,
        leaderTorqueCurrent,
        leaderStatorCurrent,
        leaderSupplyCurrent);

    ParentDevice.optimizeBusUtilizationForAll(leader);

    follower.setControl(new Follower(Constants.kElevatorLeaderId, false));
  }

  public void setPIDFSlot0(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    config.withSlot0(
        new Slot0Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKS(kS)
            .withKV(kV)
            .withKG(kG)
            .withKA(kA)
            .withGravityType(GravityTypeValue.Elevator_Static));
    leader.getConfigurator().apply(config);
  }

  public void setPIDFSlot1(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    config.withSlot1(
        new Slot1Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKS(kS)
            .withKV(kV)
            .withKG(kG)
            .withKA(kA)
            .withGravityType(GravityTypeValue.Elevator_Static));
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
  }

  public void setMmConstants(double mmKv, double mmKa) {
    config.withMotionMagic(
        new MotionMagicConfigs()
            .withMotionMagicExpo_kA(mmKa)
            .withMotionMagicExpo_kV(mmKv)
            .withMotionMagicAcceleration(0)
            .withMotionMagicCruiseVelocity(0));
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
  }

  @Override
  public void runPosition(double position, int slot) {
    positionSetpoint = position;
    leader.setControl(positionRequest.withPosition(positionSetpoint).withSlot(slot));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                reference,
                leaderTorqueCurrent,
                leaderStatorCurrent,
                leaderSupplyCurrent,
                leaderVoltage,
                leaderTemp)
            .isOK();
    boolean followerConnected =
        BaseStatusSignal.refreshAll(
                followerVoltage,
                followerTorqueCurrent,
                followerSupplyCurrent,
                followerStatorCurrent,
                followerTemp)
            .isOK();

    inputs.leaderConnected = leaderConnectedDebouncer.calculate(connected);
    inputs.followerConnected = followerConnectedDebouncer.calculate(followerConnected);
    inputs.position = position.getValueAsDouble();
    inputs.velocity = velocity.getValueAsDouble();
    inputs.reference = reference.getValueAsDouble();
    inputs.leaderVoltage = leaderVoltage.getValueAsDouble();
    inputs.leaderTorqueCurrent = leaderTorqueCurrent.getValueAsDouble();
    inputs.leaderStatorCurrent = leaderStatorCurrent.getValueAsDouble();
    inputs.leaderSupplyCurrent = leaderSupplyCurrent.getValueAsDouble();
    inputs.leaderTemp = leaderTemp.getValueAsDouble();
    inputs.followerVoltage = followerVoltage.getValueAsDouble();
    inputs.followerTorqueCurrent = followerTorqueCurrent.getValueAsDouble();
    inputs.followerStatorCurrent = followerStatorCurrent.getValueAsDouble();
    inputs.followerSupplyCurrent = followerSupplyCurrent.getValueAsDouble();
    inputs.followerTemp = followerTemp.getValueAsDouble();
    inputs.appliedPosition = positionRequest.Position;
  }

  @Override
  public void runVolts(double volts) {
    leader.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public boolean setCoastMode(boolean enabled) {
    return leader.setNeutralMode(enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake).isOK()
        & follower.setNeutralMode(enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake).isOK();
  }

  @Override
  public boolean seedPosition(double newPosition) {
    return leader.setPosition(newPosition).isOK();
  }

  @Override
  public List<TalonFX> getTalonFXs() {
    return List.of(leader, follower);
  }
}
