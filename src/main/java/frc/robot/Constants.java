package frc.robot;

import com.ctre.phoenix6.CANBus;

public class Constants {
    public static final CANBus kSwerveCanBus = new CANBus("swerve");
    public static final CANBus kManipulatorCanBus = new CANBus("rio");

    public static final int kFunnelMotorId = 0;
    public static final int kElevatorMotorId = 0;
    public static final int kManipulatorMotorId = 0;

}
