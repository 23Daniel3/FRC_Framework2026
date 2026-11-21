package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import org.ciprix.interfaces.motor.MotorIOSim;
import org.ciprix.interfaces.motor.advanced.MotorController;
import org.ciprix.sim.SimMechanism;

/**
 * Simulated shooter: the flywheel (two Kraken X60, leader and follower, on the same shaft) and the
 * kicker (NEO Vortex on a SPARK Flex), with the same configurations and names used on the robot.
 * The flywheel gains tuned on the robot reach the requested RPM here too.
 */
public class ShooterIOSim implements ShooterIO {

  // What the mechanisms are made of. Estimates: replace them with the CAD values when known.
  private static final double FLYWHEEL_GEARING = 1.0;
  private static final double FLYWHEEL_INERTIA_KG_M2 = 0.004;
  private static final double KICKER_GEARING = 1.0;
  private static final double KICKER_INERTIA_KG_M2 = 0.0008;
  // Bearings and belts: what makes a free mechanism draw a little current.
  private static final double FLYWHEEL_FRICTION_NM = 0.03;
  private static final double KICKER_FRICTION_NM = 0.02;

  private final MotorIOSim leader =
      MotorIOSim.talonFX(
              "ShooterLeaderMotor",
              ShooterConstants.MOTOR_LEADER_CONFIG,
              SimMechanism.flywheel(
                      DCMotor.getKrakenX60(2), FLYWHEEL_GEARING, FLYWHEEL_INERTIA_KG_M2)
                  .withFriction(FLYWHEEL_FRICTION_NM, 0.0))
          .withFollowers(1);

  private final MotorIOSim kicker =
      MotorIOSim.spark(
          "ShooterKickerMotor",
          ShooterConstants.KICKER_MOTOR_CONFIG,
          SimMechanism.roller(DCMotor.getNeoVortex(1), KICKER_GEARING, KICKER_INERTIA_KG_M2)
              .withFriction(KICKER_FRICTION_NM, 0.0));

  @Override
  public void updateInputs(ShooterIOInputsAutoLogged inputs) {
    leader.updateInputs(inputs.leaderInputs);
    // The follower turns with the leader: same shaft, same current.
    leader.updateFollowerInputs(inputs.followerInputs);
    kicker.updateInputs(inputs.kickerInputs);
  }

  @Override
  public MotorController controlFlywheel() {
    return leader.getMotorController();
  }

  @Override
  public MotorController controlKicker() {
    return kicker.getMotorController();
  }
}
