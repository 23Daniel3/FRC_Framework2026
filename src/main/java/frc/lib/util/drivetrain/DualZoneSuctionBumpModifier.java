package frc.lib.util.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.zones.Polygon2d;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DualZoneSuctionBumpModifier implements TranslationModifier {

  private final ZoneData zoneA;
  private final ZoneData zoneB;

  public DualZoneSuctionBumpModifier(Polygon2d zoneA, Polygon2d zoneB, double strength) {
    this.zoneA = new ZoneData(zoneA, strength);
    this.zoneB = new ZoneData(zoneB, strength);
  }

  @Override
  public Translation2d apply(Drivetrain drivetrain, Translation2d input) {
    Translation2d robotLoc = drivetrain.getPose().getTranslation();

    // Prioridade: primeira zona que contém o robô
    if (zoneA.zone.contains(robotLoc)) {
      return zoneA.apply(robotLoc);
    }

    if (zoneB.zone.contains(robotLoc)) {
      return zoneB.apply(robotLoc);
    }

    // Saiu de todas as zonas → reseta estado
    zoneA.reset();
    zoneB.reset();

    return input;
  }

  /** Classe interna que encapsula o comportamento original por zona */
  private static class ZoneData {
    private final Polygon2d zone;
    private final double strength;
    private Double forwardDirectionX = null;

    private ZoneData(Polygon2d zone, double strength) {
      this.zone = zone;
      this.strength = strength;
    }

    private Translation2d apply(Translation2d robotLoc) {
      Translation2d center = zone.getCenter();

      if (forwardDirectionX == null) {
        forwardDirectionX = (robotLoc.getX() > center.getX()) ? -1.0 : 1.0;
      }

      Translation2d pullVec = new Translation2d(forwardDirectionX * strength, 0);

      // Clamp de segurança (mesmo da versão original)
      if (pullVec.getNorm() > strength) {
        pullVec = pullVec.div(pullVec.getNorm()).times(strength);
      }

      return pullVec;
    }

    private void reset() {
      forwardDirectionX = null;
    }
  }
}
