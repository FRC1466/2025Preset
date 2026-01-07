// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.autos.DriveForward;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MirrorUtil;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  @Getter private Drive drive;
  @Getter private Vision vision;

  private Command blankAuto = Commands.none();

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert noAuto = new Alert("Please select an auto routine.", AlertType.kWarning);
  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case COMPBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  compCameras.values().stream()
                      .map(
                          config -> new VisionIOPhotonVision(config.name(), config.robotToCamera()))
                      .toArray(VisionIO[]::new));
        }
        case DEVBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  devCameras.values().stream()
                      .map(
                          config -> new VisionIOPhotonVision(config.name(), config.robotToCamera()))
                      .toArray(VisionIO[]::new));
        }
        case SIMBOT -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(TunerConstants.FrontLeft),
                  new ModuleIOSim(TunerConstants.FrontRight),
                  new ModuleIOSim(TunerConstants.BackLeft),
                  new ModuleIOSim(TunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  simCameras.values().stream()
                      .map(
                          config ->
                              new VisionIOPhotonVisionSim(
                                  config.name(), config.robotToCamera(), drive::getPose))
                      .toArray(VisionIO[]::new));
        }
      }
    }

    // No-op implementations for replay or if not set above
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (vision == null) {
      vision =
          new Vision(
              drive::addVisionMeasurement,
              cameras.values().stream().map(config -> new VisionIO() {}).toArray(VisionIO[]::new));
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    LoggedDashboardChooser<Boolean> mirror = new LoggedDashboardChooser<>("Processor Side?");
    mirror.addDefaultOption("Yes", false);
    mirror.addOption("No", true);
    LoggedDashboardChooser<Boolean> push = new LoggedDashboardChooser<>("Pushing?");
    push.addDefaultOption("No", false);
    push.addOption("Yes", true);
    MirrorUtil.setMirror(mirror::get);

    autoChooser.addDefaultOption("None", blankAuto);

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

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    LoggedTunableNumber P = new LoggedTunableNumber("Autos/P", 0);
    LoggedTunableNumber D = new LoggedTunableNumber("Autos/D", 0);

    DoubleSupplier driverX = () -> -controller.getLeftY();
    DoubleSupplier driverY = () -> -controller.getLeftX();
    DoubleSupplier driverOmega = () -> -controller.getRightX();

    Supplier<Command> joystickDriveCommandFactory =
        () -> DriveCommands.joystickDrive(drive, driverX, driverY, driverOmega);
    drive.setDefaultCommand(joystickDriveCommandFactory.get());

    controller
        .a()
        .whileTrue(
            Commands.defer(
                () -> new DriveForward(this, P::getAsDouble, D::getAsDouble).getCommand(),
                Set.of(drive)));

    controller.b().whileTrue(AutoBuilder.buildAuto("Example Auto"));

    controller
        .x()
        .whileTrue(
            AutoBuilder.pathfindToPoseFlipped(
                new Pose2d(14.5, 5, new Rotation2d(2 * Math.PI)),
                new PathConstraints(4, 3, 360.0, 720.0)));

    // Reset gyro
    var driverStartAndBack = controller.start().and(controller.back());
    driverStartAndBack.onTrue(
        Commands.runOnce(
                () ->
                    drive.setPose(
                        new Pose2d(
                            drive.getPose().getTranslation(),
                            AllianceFlipUtil.apply(Rotation2d.kZero))))
            .withName("Reset Gyro")
            .ignoringDisable(true));

    // Endgame alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(controllerRumbleCommand().withName("Controller Endgame Alert 1"));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            controllerRumbleCommand().withName("Controller Endgame Alert 2")); // Rumble three times
  }

  // Creates controller rumble command
  public Command controllerRumbleCommand() {
    return Commands.startEnd(
            () -> {
              controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
            },
            () -> {
              controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            })
        .withName("Controller Rumble");
  }

  // Update dashboard data
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(!DriverStation.isJoystickConnected(controller.getHID().getPort()));

    // Auto alert
    noAuto.set(
        DriverStation.isAutonomous()
            && !DriverStation.isEnabled()
            && autoChooser.get() == blankAuto);
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
