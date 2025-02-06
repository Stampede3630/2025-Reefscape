package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Frequency;

import static edu.wpi.first.units.Units.Hertz;

public class Constants {
    public static final CANBus kSwerveCanBus = new CANBus("Swerve");
    public static final CANBus kManipulatorCanBus = new CANBus("rio");

    public static final int kFunnelMotorId = 0;
    public static final int kClimberMotorId = 0;

    public static final int kTransferMotorId = 0;
    public static final int kElevatorLeaderId = 17;
    public static final int kElevatorFollowerId = 18;
    public static final int kManipulatorMotorId = 0;
    public static final int kCanRangeId = 0;

    public static final int kElevatorEncoderId = 0;
    public static final Frequency kNonSwerveUpdateRate = Hertz.of(250);
}
