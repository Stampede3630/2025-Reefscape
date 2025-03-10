// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.limelightPose;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.NamedCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorIO;
import frc.robot.subsystems.manipulator.ManipulatorIOTalonFX;
import frc.robot.subsystems.vision.*;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final RobotState robotState = RobotState.getInstance();
  @AutoLogOutput private int autoScoreBranch = 0;
  @AutoLogOutput private FieldConstants.ReefLevel autoScoreReefLevel = FieldConstants.ReefLevel.L4;
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController buttonBoard = new CommandXboxController(1);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Elevator elevator;
  private final Manipulator manipulator;
  private final Climber climber;
  private final LoggedNetworkNumber outtakeSpeed =
      new LoggedNetworkNumber("Manipulator/outtakeVelocity", 10);
  private final LoggedNetworkNumber intakeSpeed =
      new LoggedNetworkNumber("Manipulator/intakeVelocity", 10);
  private final LoggedNetworkNumber climberTorqueCurrent =
      new LoggedNetworkNumber("Climber/torqueCurrent", 10);
  private final LoggedNetworkNumber driveTc =
      new LoggedNetworkNumber("Drive/torqueCurrentSetpoint", 10);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                RobotState.getInstance()::addVisionObservation,
                new VisionIOLimelight(limelightPose, camera0Name, robotState::getRotation));
        elevator = new Elevator(new ElevatorIOTalonFX());
        manipulator = new Manipulator(new ManipulatorIOTalonFX());
        climber = new Climber(new ClimberIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                RobotState.getInstance()::addVisionObservation,
                new VisionIOPhotonVisionSim(
                    camera0Name, limelightPose, RobotState.getInstance()::getEstimatedPose));
        elevator = new Elevator(new ElevatorIO() {});
        manipulator = new Manipulator(new ManipulatorIO() {});
        climber = new Climber(new ClimberIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                RobotState.getInstance()::addVisionObservation,
                new VisionIO() {},
                new VisionIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        manipulator = new Manipulator(new ManipulatorIO() {});
        climber = new Climber(new ClimberIO() {});
        break;
    }

    new NamedCommands(drive, climber, elevator, manipulator, vision);
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption(
        "Drive SysId SPINNY(Quasistatic Forward)",
        drive.sysIdSpinnyQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId SPINNY(Quasistatic Reverse)",
        drive.sysIdSpinnyQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId SPINNY(Dynamic Forward)",
        drive.sysIdSpinnyDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId SPINNY(Dynamic Reverse)",
        drive.sysIdSpinnyDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick} or {@link
   * XboxController}), and then passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //     Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // ELEVATOR
    controller
        .a()
        .onTrue(
            elevator.setPosition(
                () ->
                    switch (autoScoreReefLevel) {
                      case L1 -> 18;
                      case L2 -> 20;
                      case L3 -> 36;
                      case L4 -> 60;
                    }));
    controller
        .povUp()
        .whileTrue(
            elevator
                .upCommand()
                .andThen(Commands.runOnce(() -> Leds.getInstance().climbing = true)));
    controller.povDown().whileTrue(elevator.downCommand());

    // MANIPULATOR
    controller.rightTrigger().whileTrue(manipulator.runVelocity(outtakeSpeed::get));
    controller
        .leftTrigger()
        .whileTrue(elevator.setPosition(() -> 1).andThen(manipulator.autoIntake()));

    controller.start().whileTrue(manipulator.runVelocity(() -> -outtakeSpeed.get()));

    // L1
    buttonBoard
        .axisGreaterThan(0, .90)
        .onTrue(Commands.runOnce(() -> autoScoreReefLevel = FieldConstants.ReefLevel.L1)); // 18
    // L2
    buttonBoard
        .axisLessThan(1, -.9)
        .onTrue(Commands.runOnce(() -> autoScoreReefLevel = FieldConstants.ReefLevel.L2)); // 20
    // L3
    buttonBoard
        .axisLessThan(0, -.9)
        .onTrue(Commands.runOnce(() -> autoScoreReefLevel = FieldConstants.ReefLevel.L3)); // 36
    // L4
    buttonBoard
        .axisGreaterThan(1, .9)
        .onTrue(Commands.runOnce(() -> autoScoreReefLevel = FieldConstants.ReefLevel.L4)); // 60
    for (int i = 1; i < 13; i++) {
      int finalI = i - 1;
      buttonBoard
          .button(i)
          .onTrue(Commands.runOnce(() -> autoScoreBranch = finalI >= 4 ? finalI - 4 : finalI + 8));
    }

    //    // Lock to 0° when A button is held
    //    controller
    //            .a()
    //            .whileTrue(
    //                    DriveCommands.joystickDriveAtAngle(
    //                            drive,
    //                            () -> -controller.getLeftY(),
    //                            () -> -controller.getLeftX(),
    //                            () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                    () ->
                        robotState.resetPose(
                            new Pose2d(
                                robotState.getEstimatedPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    controller.povLeft().whileTrue(climber.runTorqueCurrent(climberTorqueCurrent::get));
    controller.povRight().whileTrue(climber.runTorqueCurrent(() -> -climberTorqueCurrent.get()));

    controller
        .rightBumper()
        .whileTrue(
            AutoScore.getAutoDrive(
                drive,
                () ->
                    Optional.of(
                        new FieldConstants.CoralObjective(autoScoreBranch, autoScoreReefLevel)),
                () -> autoScoreReefLevel,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));
    manipulator
        .funnelTof()
        .onTrue(
            elevator.setPositionBlocking(() -> 0, Seconds.of(2)).andThen(manipulator.autoIntake()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
