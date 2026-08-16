package frc.lib.interfaces.motor;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.basic.BasicMotorBase;
import frc.lib.interfaces.motor.basic.BasicMotorConfig;

/**
 * Basic MotorIO implementation for CTRE Victor SPX (Phoenix 5). The Victor SPX does not have an
 * internal encoder port.
 */
public class MotorIOVictorSPX extends BasicMotorBase {

  private final VictorSPX motor;
  private final BasicMotorIOInputs inputs = new BasicMotorIOInputs();

  public MotorIOVictorSPX(String name, int id, BasicMotorConfig config) {
    super(name, config);
    this.motor = new VictorSPX(id);

    VictorSPXConfiguration spxConfig = new VictorSPXConfiguration();
    spxConfig.peakOutputForward = config.maxOutput;
    spxConfig.peakOutputReverse = config.minOutput;

    motor.configAllSettings(spxConfig);
    motor.setInverted(config.inverted);
    motor.setNeutralMode(config.brakeMode ? NeutralMode.Brake : NeutralMode.Coast);

    // Follower
    if (config.leaderMotorID != 0) {
      motor.set(ControlMode.Follower, config.leaderMotorID);
      motor.setInverted(
          config.followerInverted ? InvertType.OpposeMaster : InvertType.FollowMaster);
    }
  }

  @Override
  protected void updateHardwareInputs(BasicMotorIOInputs inputs) {
    inputs.percentOutput = motor.getMotorOutputPercent();
    inputs.appliedVolts = edu.wpi.first.units.Units.Volts.of(motor.getMotorOutputVoltage());
    inputs.isConnected = motor.getLastError() == ErrorCode.OK;
  }

  @Override
  public void runVoltage(Voltage volts) {
    currentMode = MotorControlMode.VOLTAGE;
    motor.set(ControlMode.PercentOutput, mapVoltage(volts.in(Volts)) / 12.0);
  }

  @Override
  public void runPercentOutput(double percent) {
    currentMode = MotorControlMode.PERCENT;
    motor.set(ControlMode.PercentOutput, mapOutput(percent));
  }

  @Override
  public void stop() {
    currentMode = MotorControlMode.IDLE;
    motor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    motor.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
  }

  @Override
  public void setCurrentLimit(Current current) {
    // Victor SPX does not support native supply current limiting via API.
  }

  @Override
  public BasicMotorIOInputs getMotorIOInputs() {
    return inputs;
  }
}
