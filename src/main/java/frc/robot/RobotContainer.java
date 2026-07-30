package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.game.AllianceManager;
import frc.game.FieldConstants.Poses;
import frc.game.FieldConstants.Zones;
import frc.lib.controller.NaturalXboxController;
import frc.lib.controller.VibrateXboxController;
import frc.lib.util.AllianceSelector;
import frc.lib.util.ConstantsLogger;
import frc.robot.commands.auto.AutoTrajectories;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.factories.DrivetrainCommands;
import frc.robot.commands.factories.LedCommands;
import frc.robot.commands.factories.SuperStructureCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.conveyor.*;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.led.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.vision.VisionConstants.VisionCamera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Conveyor conveyor;
  private final Drivetrain drivetrain;
  private final Intake intake;
  private final Shooter shooter;
  private final Led led;
  private final SuperStructure superStructure;
  private final Vision vision;

  public final AutoTrajectories auto;

  private final Trigger isAtBump;
  // Controller
  private final NaturalXboxController driverController = new NaturalXboxController(0);
  private final NaturalXboxController operatorController = new NaturalXboxController(1);

  private final Telemetry driveLogger;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Garante que o seletor de alianca (fallback sem FMS) aparece no dashboard desde o boot.
    AllianceSelector.getInstance();
    drivetrain = TunerConstants.createDrivetrain();
    switch (Constants.currentMode) {
      case REAL:

        // Real robot, instantiate hardware IO implementations
        conveyor = new Conveyor(new ConveyorIOHardware());
        intake = new Intake(new IntakeIOHardware());
        shooter = new Shooter(new ShooterIOHardware());
        led = new Led(new LedIOReal());
        vision =
            new Vision(
                drivetrain::addVisionMeasurement,
                () -> drivetrain.getState().Speeds,
                () -> drivetrain.getRotation().getDegrees(),
                new VisionIOLimelight("limelight-left", VisionCamera.LEFT),
                new VisionIOLimelight("limelight-front", VisionCamera.FRONT),
                new VisionIOLimelight("limelight-right", VisionCamera.RIGHT));
        break;

      case SIM:

        // Sim robot, instantiate physics sim IO implementations
        conveyor = new Conveyor(new ConveyorIO() {});
        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIO() {});
        led = new Led(new LedIOSim());
        vision =
            new Vision(
                drivetrain::addVisionMeasurement,
                () -> drivetrain.getState().Speeds,
                () -> drivetrain.getRotation().getDegrees(),
                new VisionIO() {});
        DriverStation.silenceJoystickConnectionWarning(true);
        break;

      default:

        // Replayed robot, disable IO implementations
        conveyor = new Conveyor(new ConveyorIO() {});
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        led = new Led(new LedIOSim());
        vision =
            new Vision(
                drivetrain::addVisionMeasurement,
                () -> drivetrain.getState().Speeds,
                () -> drivetrain.getRotation().getDegrees(),
                new VisionIO() {});
        break;
    }

    superStructure = new SuperStructure(conveyor, drivetrain, intake, shooter);

    auto = new AutoTrajectories(drivetrain, superStructure);

    driveLogger = new Telemetry(DrivetrainConstants.MAX_SPEED);

    isAtBump = new Trigger(() -> Zones.isAtBump(drivetrain.getPose().getTranslation()));

    // Configure the button bindings
    configureButtonBindings();
    triggersActions();

    ConstantsLogger.logConstants(Zones.class, "Zones");
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default Command
    drivetrain.setDefaultCommand(
        DrivetrainCommands.joystickDrive(
            drivetrain,
            driverController.getLeftYSupplier(),
            driverController.getLeftXSupplier(),
            driverController.getRightXSupplier()));

    // Subsystem Default Commands (Relay targets computed by SuperStructure)
    intake.setDefaultCommand(IntakeCommands.defaultCommand(superStructure, intake));
    shooter.setDefaultCommand(ShooterCommands.defaultCommand(superStructure, shooter));
    conveyor.setDefaultCommand(ConveyorCommands.defaultCommand(superStructure, conveyor));

    // A "cara" do robo e um observador do estado, definido em LedCommands.STATE_EFFECTS.
    led.setDefaultCommand(LedCommands.followRobotState(led, superStructure::getRobotState));

    superStructure.setDefaultCommand(
        SuperStructureCommands.manageRequests(
            superStructure,
            driverController.leftBumper(),
            driverController.rightBumper(),
            driverController.povLeft()));

    driverController
        .start()
        .onTrue(
            new InstantCommand(
                () ->
                    drivetrain.resetPose(
                        AllianceManager.getInstance().isBlue()
                            ? Poses.RESET_POSE_BLUE
                            : Poses.RESET_POSE_RED)));

    driverController
        .rightBumper()
        .whileTrue(
            new JoystickDriveShooting(
                drivetrain,
                superStructure,
                driverController.getLeftYSupplier(),
                driverController.getLeftXSupplier()));

    // Operator Controller (Piloto 2) - Manual Overrides
    // Interrompem apenas o subsistema especifico sem afetar o restante da SuperStructure.
    operatorController.a().whileTrue(IntakeCommands.in(intake));
    operatorController
        .b()
        .and(operatorController.start().negate())
        .whileTrue(IntakeCommands.out(intake));
    operatorController
        .x()
        .and(operatorController.start().negate())
        .whileTrue(ShooterCommands.reverse(shooter));
    operatorController.y().whileTrue(ConveyorCommands.reverse(conveyor));

    Trigger invertButton = operatorController.leftStick();

    invertButton.whileTrue(new VibrateXboxController(operatorController).continuous(0, 1, 3, true));

    operatorController
        .start()
        .and(operatorController.x())
        .onTrue(AllianceManager.getInstance().setBlueStartsScoring());

    operatorController
        .start()
        .and(operatorController.b())
        .onTrue(AllianceManager.getInstance().setRedStartsScoring());

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    drivetrain.registerTelemetry(driveLogger::telemeterize);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auto.auto();
  }

  public void triggersActions() {
    // Em disabled: vermelho persistente se alguma camera lateral ve 2 tags, verde caso
    // contrario. Comandos persistentes (run) interrompem o followRobotState enquanto ativos
    // e o devolvem ao terminar — os antigos runOnce eram desfeitos no ciclo seguinte.
    Trigger twoTags =
        new Trigger(() -> vision.getTagCount(VisionCamera.LEFT) == 2)
            .or(() -> vision.getTagCount(VisionCamera.RIGHT) == 2);

    twoTags
        .and(RobotModeTriggers.disabled())
        .whileTrue(LedCommands.solidPersistent(led, edu.wpi.first.wpilibj.util.Color.kRed));

    twoTags
        .negate()
        .and(RobotModeTriggers.disabled())
        .whileTrue(LedCommands.solidPersistent(led, edu.wpi.first.wpilibj.util.Color.kGreen));

    isAtBump
        .and(RobotModeTriggers.autonomous().negate())
        .whileTrue(
            DrivetrainCommands.joystickDriveTrench(
                drivetrain,
                driverController.getLeftYSupplier(),
                driverController.getLeftXSupplier()));
  }
}
