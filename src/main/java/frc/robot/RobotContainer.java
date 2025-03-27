// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.limelightPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ButtonBoard;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private boolean hasRunAutoOnceBefore = false;
  private final RobotState robotState = RobotState.getInstance();
  private final Leds leds = Leds.getInstance();
  private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter angularSlewRateLimiter = new SlewRateLimiter(10);
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final ButtonBoard buttonBoard = new ButtonBoard(1, "SmartDashboard/ButtonBoard");
  // Dashboard inputs
  private final LoggedDashboardChooser<PathPlannerAuto> autoChooser;
  private final Elevator elevator;
  private final Manipulator manipulator;
  private final Climber climber;
  private final LoggedNetworkNumber outtakeSpeedL4 =
      new LoggedNetworkNumber("SmartDashboard/Manipulator/outtakeVelocityL4", 25);
  private final LoggedNetworkNumber outtakeSpeed =
      new LoggedNetworkNumber("SmartDashboard/Manipulator/outtakeVelocity", 18);
  private final LoggedNetworkNumber intakeSpeed =
      new LoggedNetworkNumber("SmartDashboard/Manipulator/intakeVelocity", 10);
  private final LoggedNetworkNumber climberTorqueCurrent =
      new LoggedNetworkNumber("SmartDashboard/Climber/torqueCurrent", 300);
  private final LoggedNetworkNumber driveTc =
      new LoggedNetworkNumber("SmartDashboard/Drive/torqueCurrentSetpoint", 10);
  private final LoggedNetworkNumber l1Offset =
      new LoggedNetworkNumber("SmartDashboard/ElevatorOffsets/L1", 0);
  private final LoggedNetworkNumber l2Offset =
      new LoggedNetworkNumber("SmartDashboard/ElevatorOffsets/L2", 0);
  private final LoggedNetworkNumber l3Offset =
      new LoggedNetworkNumber("SmartDashboard/ElevatorOffsets/L3", 0);
  private final LoggedNetworkNumber l4Offset =
      new LoggedNetworkNumber("SmartDashboard/ElevatorOffsets/L4", 0);
  private final LoggedNetworkBoolean takeSnapshot =
      new LoggedNetworkBoolean("SmartDashboard/Take Snapshot before Auto", false);
  @AutoLogOutput private int autoScoreBranch = 0;
  @AutoLogOutput private FieldConstants.ReefLevel autoScoreReefLevel = FieldConstants.ReefLevel.L4;

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
                RobotState.getInstance()
                    ::addVisionObservation, // switch this out for FindCameraOffset when seeding
                // camera offsets.
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
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", buildAutoChooser(""));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        new PathPlannerAuto(DriveCommands.wheelRadiusCharacterization(drive)));
    autoChooser.addOption(
        "Drive Simple FF Characterization",
        new PathPlannerAuto(DriveCommands.feedforwardCharacterization(drive)));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        new PathPlannerAuto(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        new PathPlannerAuto(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        new PathPlannerAuto(drive.sysIdDynamic(SysIdRoutine.Direction.kForward)));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        new PathPlannerAuto(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)));

    autoChooser.addOption(
        "Drive SysId SPINNY(Quasistatic Forward)",
        new PathPlannerAuto(drive.sysIdSpinnyQuasistatic(SysIdRoutine.Direction.kForward)));
    autoChooser.addOption(
        "Drive SysId SPINNY(Quasistatic Reverse)",
        new PathPlannerAuto(drive.sysIdSpinnyQuasistatic(SysIdRoutine.Direction.kReverse)));
    autoChooser.addOption(
        "Drive SysId SPINNY(Dynamic Forward)",
        new PathPlannerAuto(drive.sysIdSpinnyDynamic(SysIdRoutine.Direction.kForward)));
    autoChooser.addOption(
        "Drive SysId SPINNY(Dynamic Reverse)",
        new PathPlannerAuto(drive.sysIdSpinnyDynamic(SysIdRoutine.Direction.kReverse)));

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
            () -> xSlewRateLimiter.calculate(-controller.getLeftY()),
            () -> ySlewRateLimiter.calculate(-controller.getLeftX()),
            () -> angularSlewRateLimiter.calculate(-controller.getRightX())));

    // ELEVATOR
    controller
        .a()
        .onTrue(
            elevator
                .setPosition(
                    () ->
                        switch (autoScoreReefLevel) {
                              case L1 -> l1Offset.get();
                              case L2 -> l2Offset.get();
                              case L3 -> l3Offset.get();
                              case L4 -> l4Offset.get();
                            }
                            + autoScoreReefLevel.height)
                .andThen());
    controller.povUp().whileTrue(elevator.upCommand());
    controller.povDown().whileTrue(elevator.downCommand());

    // MANIPULATOR
    controller
        .rightTrigger()
        .whileTrue(
            Commands.either(
                manipulator.runVelocity(
                    () ->
                        switch (autoScoreReefLevel) {
                          case L4 -> outtakeSpeedL4.get();
                          default -> outtakeSpeed.get();
                        }),
                manipulator.autoIntake(),
                manipulator.haveAGamePiece()));

    controller
        .leftTrigger()
        .whileTrue(elevator.intakeHeight())
        .onTrue(Commands.runOnce(() -> leds.isIntaking = true))
        .onFalse(Commands.runOnce(() -> leds.isIntaking = false));
    // Todo: this is where we add LEDs
    controller.start().whileTrue(manipulator.runVelocity(() -> -outtakeSpeed.get()));

    // L1
    buttonBoard
        .l1()
        .onTrue(
            Commands.runOnce(
                () -> {
                  autoScoreReefLevel = FieldConstants.ReefLevel.L1;
                })); // 18
    // L2
    buttonBoard
        .l2()
        .onTrue(Commands.runOnce(() -> autoScoreReefLevel = FieldConstants.ReefLevel.L2)); // 18
    // L3
    buttonBoard
        .l3()
        .onTrue(Commands.runOnce(() -> autoScoreReefLevel = FieldConstants.ReefLevel.L3)); // 34
    // L4
    buttonBoard
        .l4()
        .onTrue(Commands.runOnce(() -> autoScoreReefLevel = FieldConstants.ReefLevel.L4)); // 58

    for (int i = 1; i < 13; i++) {
      int finalI = i - 1;
      buttonBoard
          .button(i)
          .onTrue(
              Commands.runOnce(
                      () -> {
                        autoScoreBranch = finalI >= 4 ? finalI - 4 : finalI + 8;
                        Leds.getInstance().autoScoringLevel = autoScoreReefLevel;
                      })
                  .andThen(Commands.print("hllo")));
    }

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
    controller.povLeft().whileTrue(climber.runBangBang(climberTorqueCurrent::get, () -> -.049));
    controller.povRight().whileTrue(climber.runTorqueCurrent(() -> -climberTorqueCurrent.get()));

    //    controller.povLeft().whileTrue(climber.setPosition(() -> -.26));
    //    controller.povRight().whileTrue(climber.setPosition(() -> -.049));

    controller
        .rightBumper()
        .whileTrue(
            Commands.either(
                AutoScore.getAutoDrive( // if have a game piece, auto align
                        drive,
                        () ->
                            Optional.of(
                                new FieldConstants.CoralObjective(
                                    autoScoreBranch, autoScoreReefLevel)),
                        () -> autoScoreReefLevel,
                        () -> xSlewRateLimiter.calculate(-controller.getLeftY()),
                        () -> ySlewRateLimiter.calculate(-controller.getLeftX()),
                        () -> angularSlewRateLimiter.calculate(-controller.getRightX()))
                    .alongWith(
                        Commands.runOnce(
                            () -> {
                              leds.autoScoringLevel = autoScoreReefLevel;
                              Leds.getInstance().autoScoring = true;
                            }))
                    .andThen(Commands.runOnce(() -> Leds.getInstance().autoScoring = false)),
                DriveCommands
                    .joystickDriveAtAngle( // if don't have a game piece, lock to coral station
                        drive,
                        () -> xSlewRateLimiter.calculate(-controller.getLeftY()),
                        () -> ySlewRateLimiter.calculate(-controller.getLeftX()),
                        () -> {
                          if (FieldConstants.CoralStation.leftRegion.inRegion(
                              robotState.getEstimatedPose().getTranslation()))
                            return AllianceFlipUtil.apply(
                                FieldConstants.CoralStation.leftCenterFace.getRotation());
                          else if (FieldConstants.CoralStation.rightRegion.inRegion(
                              robotState.getEstimatedPose().getTranslation()))
                            return AllianceFlipUtil.apply(
                                FieldConstants.CoralStation.rightCenterFace.getRotation());
                          else return robotState.getEstimatedPose().getRotation();
                        }),
                manipulator
                    .haveAGamePiece()
                    .onTrue(
                        Commands.runOnce(() -> Leds.getInstance().coralGrabbed = true)
                            .andThen(Commands.waitSeconds(1))
                            .andThen(
                                Commands.runOnce(() -> Leds.getInstance().coralGrabbed = false)))
                    .onFalse(Commands.runOnce(() -> Leds.getInstance().coralGrabbed = false))));
  }

  public void runThisBeforeTele() {
    manipulator
        .funnelTof()
        .onTrue(manipulator.autoIntake().alongWith(Commands.runOnce(() -> leds.isIntaking = true)))
        .onFalse(Commands.runOnce(() -> leds.isIntaking = false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return AutoBuilder.buildAuto("P5 first part").andThen(AutoBuilder.buildAuto("P5 second
    // part"));
    // return AutoBuilder.buildAuto("P2 first part").andThen(AutoBuilder.buildAuto("P2 second
    // part"));
    PathPlannerAuto auto = autoChooser.get();

    if (hasRunAutoOnceBefore) auto = new PathPlannerAuto(autoChooser.get().getName());

    hasRunAutoOnceBefore = true;
    if (takeSnapshot.get() || DriverStation.isFMSAttached()) {
      return vision
          .takeSnapshot(
              () ->
                  DriverStation.getMatchType().toString()
                      + " "
                      + DriverStation.getMatchNumber()
                      + " "
                      + DriverStation.getRawAllianceStation())
          .andThen(
              vision.seedPoseBeforeAuto(
                  AllianceFlipUtil.apply(auto.getStartingPose()), Meters.of(1)))
          .andThen(auto)
          .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }
    return vision
        .seedPoseBeforeAuto(AllianceFlipUtil.apply(auto.getStartingPose()), Meters.of(1))
        .andThen(auto)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  public SendableChooser<PathPlannerAuto> buildAutoChooser(String defaultAutoName) {
    SendableChooser<PathPlannerAuto> chooser = new SendableChooser<>();
    List<String> autoNames = AutoBuilder.getAllAutoNames();

    PathPlannerAuto defaultOption = null;

    for (String autoName : autoNames) {
      PathPlannerAuto auto = new PathPlannerAuto(autoName);

      if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) {
        defaultOption = auto;
      } else {
        chooser.addOption(autoName, auto);
      }
    }

    if (defaultOption == null) {
      chooser.setDefaultOption("None", new PathPlannerAuto(Commands.none()));
    } else {
      chooser.setDefaultOption(defaultOption.getName(), defaultOption);
      chooser.addOption("None", new PathPlannerAuto(Commands.none()));
    }

    return chooser;
  }
}
