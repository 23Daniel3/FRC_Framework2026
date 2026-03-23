package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.MotorConfig;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOTalonFX;
import frc.lib.util.CANType;

public class FlywheelIOTalonFX implements FlywheelIO {

  private final MotorIO leader;
  private final MotorIO follower;

  public FlywheelIOTalonFX() {
    MotorConfig configLeader =
        new MotorConfig()
            .currentLimit(Amps.of(FlywheelConstants.CURRENT_LIMIT))
            .coastMode()
            .nominalVoltage(Volts.of(FlywheelConstants.NOMINAL_VOLTAGE))
            .inverted(true);

    MotorConfig configFollower =
        new MotorConfig()
            .currentLimit(Amps.of(FlywheelConstants.CURRENT_LIMIT))
            .coastMode()
            .nominalVoltage(Volts.of(FlywheelConstants.NOMINAL_VOLTAGE))
            .withMotorLeader(FlywheelConstants.LEADER_ID)
            .withMotorAlignment(MotorAlignmentValue.Opposed);

    leader = new MotorIOTalonFX(FlywheelConstants.LEADER_ID, CANType.RIO, configLeader);
    follower = new MotorIOTalonFX(FlywheelConstants.FOLLOWER_ID, CANType.RIO, configFollower);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.leaderInputs = leader.getMotorIOInputs();
    inputs.followerInputs = follower.getMotorIOInputs();
  }

  @Override
  public void runVelocity(AngularVelocity velocity) {
    leader.runVelocity(velocity, 0, Volts.of(0));
  }

  @Override
  public void runPercentOutput(double percentOutput) {
    leader.setPercentOutput(percentOutput);
  }

  @Override
  public void runVoltage(Voltage voltage) {
    leader.setVoltage(voltage);
  }

  @Override
  public void stop() {
    leader.stop();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    leader.configurePIDF(0, kP, kI, kD, 0.0);
  }

  @Override
  public void configureKSVA(double kS, double kV, double kA) {
    leader.configureKSVA(0, kS, kV, kA);
  }

  @Override
  public void setCurrentLimit(Current current) {
    leader.setCurrentLimit(current);
  }
}
