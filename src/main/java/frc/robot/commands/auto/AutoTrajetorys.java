package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.game.FieldConstants;
import frc.lib.util.AutoEngine;
import frc.robot.commands.drivetrain.align.AimHub;
import frc.robot.factories.SuperStructureCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.superstructure.SuperStructure;

public class AutoTrajetorys {
  private final AutoEngine engine;
  private final Drivetrain drivetrain;
  private final SuperStructure superStructure;

  public AutoTrajetorys(Drivetrain drivetrain, SuperStructure superStructure) {
    this.drivetrain = drivetrain;
    this.superStructure = superStructure;

    // Criamos 3 slots: [0] Posição Inicial, [1] Primeira Peça, [2] Segunda Peça/Ação
    this.engine = new AutoEngine("AutoMod", 3);

    setupOptions();
    setupFilters();
  }

  private void setupOptions() {
    // --- SLOT 0: POSIÇÕES INICIAIS ---
    engine.addDefaultPart("Início: Esquerda", () -> resetPoseStep("LEFT"));
    engine.addPart("Início: Direita", () -> resetPoseStep("RIGHT"));
    engine.addPart("Início: Meio Esq", () -> resetPoseStep("MIDDLE_LEFT"));

    // --- SLOT 1 & 2: AÇÕES ---
    engine.addPart(
        "Atirar e Coletar (Esq)",
        () ->
            Commands.deadline(
                new PathPlannerAuto("StartCollectingNeutralZoneLeftShoot"),
                SuperStructureCommands.collect(superStructure)));

    engine.addPart(
        "Atirar e Coletar (Dir)",
        () ->
            Commands.deadline(
                new PathPlannerAuto("StartCollectingNeutralZoneRightShoot"),
                SuperStructureCommands.collect(superStructure)));

    engine.addPart(
        "Mira e Atira",
        () ->
            Commands.deadline(
                new AimHub(drivetrain, superStructure).withTimeout(3),
                SuperStructureCommands.shoot(superStructure)));

    engine.addPart(
        "Ciclo Contínuo",
        () -> SuperStructureCommands.collectShooting(superStructure).until(() -> false));

    engine.addPart("Nada", Commands::none);
  }

  private void setupFilters() {
    // Exemplo de Filtro em Cascata:
    // Se o Slot 0 for "Início: Esquerda", o Slot 1 não deve mostrar "Atirar e Coletar (Dir)"
    engine
        .getChooser()
        .setFilter(
            1,
            (optionName) -> {
              String startPos = engine.getChooser().getSelectedKey(0);
              if (startPos.contains("Esquerda") && optionName.contains("(Dir)")) return false;
              if (startPos.contains("Direita") && optionName.contains("(Esq)")) return false;
              return true;
            });
  }

  private Command resetPoseStep(String side) {
    return Commands.runOnce(
        () -> {
          boolean red = DriverStation.getAlliance().get() == Alliance.Red;
          Pose2d pose =
              switch (side) {
                case "LEFT" ->
                    red
                        ? FieldConstants.Poses.START_AUTO_LEFT_RED
                        : FieldConstants.Poses.START_AUTO_LEFT_BLUE;
                case "RIGHT" ->
                    red
                        ? FieldConstants.Poses.START_AUTO_RIGHT_RED
                        : FieldConstants.Poses.START_AUTO_RIGHT_BLUE;
                default -> new Pose2d();
              };
          drivetrain.resetPose(pose);
        });
  }

  public Command auto() {
    return engine.build();
  }
}
