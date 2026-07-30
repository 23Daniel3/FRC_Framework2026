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
import frc.robot.commands.factories.SuperStructureCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.superstructure.SuperStructure;

public class AutoTrajectories {

  public enum StartPosition {
    LEFT,
    RIGHT,
    MIDDLE_LEFT,
    MIDDLE_RIGHT
  }

  private final AutoEngine engine;
  private final Drivetrain drivetrain;
  private final SuperStructure superStructure;

  public AutoTrajectories(Drivetrain drivetrain, SuperStructure superStructure) {
    this.drivetrain = drivetrain;
    this.superStructure = superStructure;

    this.engine = new AutoEngine("AutoMod", 3);

    setupOptions();
    setupFilters();
  }

  private void setupOptions() {
    // SLOT 0: POSITIONS
    engine.addDefaultPart("Inicio: Esquerda", () -> resetPoseStep(StartPosition.LEFT));
    engine.addPart("Inicio: Direita", () -> resetPoseStep(StartPosition.RIGHT));
    engine.addPart("Inicio: Meio Esq", () -> resetPoseStep(StartPosition.MIDDLE_LEFT));

    // SLOT 1 & 2: ACTIONS
    // Notice: Instantiating PathPlannerAuto inside defer() causes file read at Auto Init.
    // If loop overruns occur, pre-load the PathPlannerPath objects during RobotInit.
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
                new AimHub(drivetrain, superStructure).withTimeout(3.0),
                SuperStructureCommands.shoot(superStructure)));

    engine.addPart("Nada", Commands::none);
  }

  private void setupFilters() {
    // Brittle filter mapping. Consider moving to an AutoTag abstraction.
    engine
        .getChooser()
        .setFilter(
            1,
            (optionName) -> {
              String startPos = engine.getChooser().getSelectedKey(0);
              if (startPos == null) return true;

              if (startPos.contains("Esquerda") && optionName.contains("(Dir)")) return false;
              if (startPos.contains("Direita") && optionName.contains("(Esq)")) return false;

              return true;
            });
  }

  private Command resetPoseStep(StartPosition side) {
    return Commands.runOnce(
        () -> {
          // Safe fallback to Blue if FMS/DriverStation is missing
          boolean red = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

          Pose2d pose =
              switch (side) {
                case LEFT ->
                    red
                        ? FieldConstants.Poses.START_AUTO_LEFT_RED
                        : FieldConstants.Poses.START_AUTO_LEFT_BLUE;
                case RIGHT ->
                    red
                        ? FieldConstants.Poses.START_AUTO_RIGHT_RED
                        : FieldConstants.Poses.START_AUTO_RIGHT_BLUE;
                case MIDDLE_LEFT ->
                    red
                        ? FieldConstants.Poses.START_AUTO_MIDDLE_LEFT_RED
                        : FieldConstants.Poses.START_AUTO_MIDDLE_LEFT_BLUE;
                case MIDDLE_RIGHT ->
                    red
                        ? FieldConstants.Poses.START_AUTO_MIDDLE_RIGHT_RED
                        : FieldConstants.Poses.START_AUTO_MIDDLE_RIGHT_BLUE;
              };
          drivetrain.resetPose(pose);
        });
  }

  public Command auto() {
    return engine.build();
  }
}
