package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.lib.interfaces.motor.MotorController;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOSparkFlex;
import frc.lib.interfaces.motor.MotorIOTalonFX;

public class ShooterIOHardware implements ShooterIO {

  private final MotorIO leader;
  private final MotorIO follower;
  private final MotorIO kickerMotor;

  public ShooterIOHardware() {
    leader =
        new MotorIOTalonFX(
            "ShooterLeaderMotor",
            ShooterConstants.LEADER_ID,
            new CANBus(),
            ShooterConstants.MOTOR_LEADER_CONFIG);
    follower =
        new MotorIOTalonFX(
            "ShooterFollowerMotor",
            ShooterConstants.FOLLOWER_ID,
            new CANBus(),
            ShooterConstants.MOTOR_FOLLOWER_CONFIG);
    kickerMotor =
        new MotorIOSparkFlex(
            "ShooterKickerMotor",
            ShooterConstants.KICKER_MOTOR_ID,
            MotorType.kBrushless,
            ShooterConstants.KICKER_MOTOR_CONFIG);
  }

  @Override
  public void updateInputs(ShooterIOInputsAutoLogged inputs) {
    inputs.leaderInputs = leader.getMotorIOInputs();
    inputs.followerInputs = follower.getMotorIOInputs();
    inputs.kickerInputs = kickerMotor.getMotorIOInputs();
  }

  @Override
  public MotorController controlFlywheel() {
    return leader.getMotorController();
  }

  @Override
  public MotorController controlKicker() {
    return kickerMotor.getMotorController();
  }
}
