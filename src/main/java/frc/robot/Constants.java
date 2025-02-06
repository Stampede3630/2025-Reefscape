package frc.robot;

import com.ctre.phoenix6.CANBus;

public class Constants {
    public static final CANBus kSwerveCanBus = new CANBus("Swerve");
    public static final CANBus kManipulatorCanBus = new CANBus("rio");

    public static final int kFunnelMotorId = 0;
    public static final int kElevatorMotor1Id = 17;
    public static final int kElevatorMotor2Id = 18;

    public static final int kManipulatorMotorId = 0;

}
