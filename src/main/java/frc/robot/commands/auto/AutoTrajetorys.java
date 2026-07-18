package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.game.FieldConstants;
import frc.lib.util.AllianceSelector;
import frc.robot.commands.drivetrain.align.AimHub;
import frc.robot.factories.SuperStructureCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.superstructure.SuperStructure;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoTrajetorys {

  public final Drivetrain drivetrain;
  public final SuperStructure superStructure;

  enum sides {
    LEFT,
    RIGHT,
    MIDDLE_LEFT,
    MIDDLE_RIGHT,
  }

  private final LoggedDashboardChooser<Command> chooser =
      new LoggedDashboardChooser<>("Auto_Chooser");

  private final LoggedDashboardChooser<sides> sideChooser =
      new LoggedDashboardChooser<>("Side_Chooser");

  public AutoTrajetorys(Drivetrain drivetrain, SuperStructure superStructure) {
    this.drivetrain = drivetrain;
    this.superStructure = superStructure;

    chooser.addDefaultOption(
        "Esq - Atira > Coleta Zona Neutra > Atira", L_Shoot_IntakeNeutralZone_Shoot());

    chooser.addOption(
        "Dir - Atira > Coleta Zona Neutra > Atira", R_Shoot_IntakeNeutralZone_Shoot());

    chooser.addOption(
        "Dir - > Atira > fica coletando e atirando", R_Shoot_IntakeNeutralZoneShootingToAlliance());

    chooser.addOption(
        "So Mirar e atirar e resetar a posicao - selecionar lado de inicio", Aim_Shoot());

    sideChooser.addDefaultOption("LEFT", sides.LEFT);
    sideChooser.addOption("RIGHT", sides.RIGHT);
    sideChooser.addOption("MIDDLE_LEFT", sides.MIDDLE_LEFT);
    sideChooser.addOption("MIDDLE_RIGHT", sides.MIDDLE_RIGHT);
  }

  public Command auto() {
    return chooser.get();
  }

  public boolean isFlipped() {
    return AllianceSelector.getInstance().shouldFlip();
  }

  public SequentialCommandGroup L_Shoot_IntakeNeutralZone_Shoot() {
    return new SequentialCommandGroup(
        Commands.deadline(
            new AimHub(drivetrain, superStructure).alongWith(new WaitCommand(5)),
            SuperStructureCommands.shoot(superStructure)),
        Commands.deadline(
            new PathPlannerAuto("StartCollectingNeutralZoneLeftShoot"),
            SuperStructureCommands.collect(superStructure)),
        Commands.parallel(
            new AimHub(drivetrain, superStructure),
            SuperStructureCommands.shoot(superStructure).until(() -> false)),
        SuperStructureCommands.shoot(superStructure).until(() -> false));
  }

  public SequentialCommandGroup R_Shoot_IntakeNeutralZone_Shoot() {
    return new SequentialCommandGroup(
        Commands.deadline(
            new AimHub(drivetrain, superStructure).alongWith(new WaitCommand(5)),
            SuperStructureCommands.shoot(superStructure)),
        Commands.deadline(
            new PathPlannerAuto("StartCollectingNeutralZoneRightShoot"),
            SuperStructureCommands.collect(superStructure)),
        Commands.parallel(
            new AimHub(drivetrain, superStructure),
            SuperStructureCommands.shoot(superStructure).until(() -> false)),
        SuperStructureCommands.shoot(superStructure).until(() -> false));
  }

  public SequentialCommandGroup R_Shoot_IntakeNeutralZoneShootingToAlliance() {
    return new SequentialCommandGroup(
        Commands.deadline(
            new AimHub(drivetrain, superStructure).alongWith(new WaitCommand(5)),
            SuperStructureCommands.shoot(superStructure)),
        Commands.deadline(
            new PathPlannerAuto("CollectingNeutralZoneShotting"),
            new ParallelCommandGroup(
                SuperStructureCommands.collectShooting(superStructure).until(() -> false))),
        new ParallelCommandGroup(
            SuperStructureCommands.collectShooting(superStructure).until(() -> false)));
  }

  public Command Aim_Shoot() {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              sides side = sideChooser.get();
              boolean flipped = isFlipped();

              switch (side) {
                case LEFT:
                  drivetrain.resetPose(
                      flipped
                          ? FieldConstants.Poses.START_AUTO_LEFT_RED
                          : FieldConstants.Poses.START_AUTO_LEFT_BLUE);
                  break;
                case RIGHT:
                  drivetrain.resetPose(
                      flipped
                          ? FieldConstants.Poses.START_AUTO_RIGHT_RED
                          : FieldConstants.Poses.START_AUTO_RIGHT_BLUE);
                  break;
                case MIDDLE_LEFT:
                  drivetrain.resetPose(
                      flipped
                          ? FieldConstants.Poses.START_AUTO_MIDDLE_LEFT_RED
                          : FieldConstants.Poses.START_AUTO_MIDDLE_LEFT_BLUE);
                  break;
                case MIDDLE_RIGHT:
                  drivetrain.resetPose(
                      flipped
                          ? FieldConstants.Poses.START_AUTO_MIDDLE_RIGHT_RED
                          : FieldConstants.Poses.START_AUTO_MIDDLE_RIGHT_BLUE);
                  break;
              }
            }),
        Commands.deadline(
            new AimHub(drivetrain, superStructure).alongWith(new WaitCommand(10)),
            SuperStructureCommands.shoot(superStructure)));
  }
}
