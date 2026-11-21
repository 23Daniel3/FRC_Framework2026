package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import org.ciprix.interfaces.motor.MotorIOSparkFlex;
import org.ciprix.interfaces.motor.MotorIOTalonFX;
import org.ciprix.interfaces.motor.advanced.MotorController;
import org.ciprix.interfaces.motor.advanced.MotorIO;

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
    leader.updateInputs(inputs.leaderInputs);
    follower.updateInputs(inputs.followerInputs);
    kickerMotor.updateInputs(inputs.kickerInputs);
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
