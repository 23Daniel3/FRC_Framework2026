package frc.lib.calculus;

import edu.wpi.first.math.geometry.Rotation2d;

public record ShotParameters(Rotation2d aimAngle, double rpm) {

  public static final ShotParameters IDLE = new ShotParameters(new Rotation2d(), 0.0);
}