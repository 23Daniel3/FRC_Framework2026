package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.game.FieldConstants;
import frc.lib.util.AllianceSelector;
import frc.robot.commands.CommandConstants.AutoConstants;
import frc.robot.commands.drivetrain.align.AimHub;
import frc.robot.factories.SuperStructureCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.superstructure.SuperStructure;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Registro e composicao dos autonomos.
 *
 * <p>Padroes usados aqui:
 *
 * <ul>
 *   <li>{@code aimAndShoot(min)} — mira no hub E atira por pelo menos {@code min} segundos (o
 *       deadline termina quando a mira converge e o tempo minimo passa);
 *   <li>Comandos de request da SuperStructure ({@code shoot}, {@code collect}...) sao {@code
 *       startEnd} e nunca terminam sozinhos — como ultimo passo de um auto, rodam ate o fim do
 *       periodo autonomo (nao e necessario {@code .until(...)});
 *   <li>Autos espelhados diferem apenas no nome do path — use os metodos parametrizados em vez de
 *       duplicar a sequencia.
 * </ul>
 */
public class AutoTrajectories {

  public final Drivetrain drivetrain;
  public final SuperStructure superStructure;

  enum Sides {
    LEFT,
    RIGHT,
    MIDDLE_LEFT,
    MIDDLE_RIGHT,
  }

  private final LoggedDashboardChooser<Command> chooser =
      new LoggedDashboardChooser<>("Auto_Chooser");

  private final LoggedDashboardChooser<Sides> sideChooser =
      new LoggedDashboardChooser<>("Side_Chooser");

  public AutoTrajectories(Drivetrain drivetrain, SuperStructure superStructure) {
    this.drivetrain = drivetrain;
    this.superStructure = superStructure;

    chooser.addDefaultOption(
        "Esq - Atira > Coleta Zona Neutra > Atira",
        shootCollectShoot("StartCollectingNeutralZoneLeftShoot"));

    chooser.addOption(
        "Dir - Atira > Coleta Zona Neutra > Atira",
        shootCollectShoot("StartCollectingNeutralZoneRightShoot"));

    chooser.addOption(
        "Dir - Atira > fica coletando e atirando",
        shootThenCollectShooting("CollectingNeutralZoneShotting"));

    chooser.addOption("So mirar e atirar (selecionar lado de inicio)", aimShootOnly());

    sideChooser.addDefaultOption("LEFT", Sides.LEFT);
    sideChooser.addOption("RIGHT", Sides.RIGHT);
    sideChooser.addOption("MIDDLE_LEFT", Sides.MIDDLE_LEFT);
    sideChooser.addOption("MIDDLE_RIGHT", Sides.MIDDLE_RIGHT);
  }

  public Command auto() {
    return chooser.get();
  }

  public boolean isFlipped() {
    return AllianceSelector.getInstance().shouldFlip();
  }

  /** Mira no hub e atira ate a mira convergir E {@code minTimeSec} se passar. */
  private Command aimAndShoot(double minTimeSec) {
    return Commands.deadline(
        new AimHub(drivetrain, superStructure).alongWith(Commands.waitSeconds(minTimeSec)),
        SuperStructureCommands.shoot(superStructure));
  }

  /** Atira > segue o path coletando > mira e atira ate o fim do autonomo. */
  private SequentialCommandGroup shootCollectShoot(String pathName) {
    return new SequentialCommandGroup(
        aimAndShoot(AutoConstants.AIM_SHOOT_MIN_TIME_SEC),
        Commands.deadline(
            new PathPlannerAuto(pathName), SuperStructureCommands.collect(superStructure)),
        // Ultimo passo: nunca termina (roda ate o fim do periodo autonomo).
        Commands.parallel(
            new AimHub(drivetrain, superStructure), SuperStructureCommands.shoot(superStructure)));
  }

  /** Atira > segue o path em modo coletar+atirar > continua coletando+atirando ate o fim. */
  private SequentialCommandGroup shootThenCollectShooting(String pathName) {
    return new SequentialCommandGroup(
        aimAndShoot(AutoConstants.AIM_SHOOT_MIN_TIME_SEC),
        Commands.deadline(
            new PathPlannerAuto(pathName), SuperStructureCommands.collectShooting(superStructure)),
        // Ultimo passo: nunca termina (roda ate o fim do periodo autonomo).
        SuperStructureCommands.collectShooting(superStructure));
  }

  /** Reseta a pose pelo lado escolhido no dashboard e apenas mira e atira. */
  private Command aimShootOnly() {
    return new SequentialCommandGroup(
        new InstantCommand(this::resetPoseFromSideChooser),
        aimAndShoot(AutoConstants.AIM_ONLY_SHOOT_TIME_SEC));
  }

  private void resetPoseFromSideChooser() {
    Sides side = sideChooser.get();
    boolean flipped = isFlipped();

    switch (side) {
      case LEFT ->
          drivetrain.resetPose(
              flipped
                  ? FieldConstants.Poses.START_AUTO_LEFT_RED
                  : FieldConstants.Poses.START_AUTO_LEFT_BLUE);
      case RIGHT ->
          drivetrain.resetPose(
              flipped
                  ? FieldConstants.Poses.START_AUTO_RIGHT_RED
                  : FieldConstants.Poses.START_AUTO_RIGHT_BLUE);
      case MIDDLE_LEFT ->
          drivetrain.resetPose(
              flipped
                  ? FieldConstants.Poses.START_AUTO_MIDDLE_LEFT_RED
                  : FieldConstants.Poses.START_AUTO_MIDDLE_LEFT_BLUE);
      case MIDDLE_RIGHT ->
          drivetrain.resetPose(
              flipped
                  ? FieldConstants.Poses.START_AUTO_MIDDLE_RIGHT_RED
                  : FieldConstants.Poses.START_AUTO_MIDDLE_RIGHT_BLUE);
    }
  }
}
