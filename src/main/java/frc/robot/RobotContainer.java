// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController buttonBoard = new CommandXboxController(1);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    /* Subsystems */
    private final Elevator m_elevator = new Elevator();
    private final Funnel m_funnel = new Funnel();
    private final Manipulator m_manipulator = new Manipulator();
    private final Climber m_climber = new Climber();
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting upCommand bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    public RobotContainer() {
        SignalLogger.enableAutoLogging(true);
        configureBindings();

        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Note that each routine should be run exactly once in a single log.
        joystick.y().whileTrue(m_manipulator.sysIdDynamic(Direction.kForward));
        joystick.x().whileTrue(m_manipulator.sysIdDynamic(Direction.kReverse));
        joystick.a().whileTrue(m_manipulator.sysIdQuasistatic(Direction.kForward));
        joystick.b().whileTrue(m_manipulator.sysIdQuasistatic(Direction.kReverse));

        joystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start).andThen(Commands.print("Started Log")));
        joystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop).andThen(Commands.print("Stopped Log")));

        // // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // ELEVATOR
        joystick.povUp().whileTrue(m_elevator.upCommand());
        joystick.povDown().whileTrue(m_elevator.downCommand());

        // MANIPULATOR
        joystick.rightTrigger().whileTrue(m_manipulator.velocityCommand(() -> 12));
        joystick.leftTrigger().whileTrue(m_manipulator.dutyCycleCommand(joystick::getLeftTriggerAxis));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
