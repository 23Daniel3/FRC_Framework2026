package org.ciprix.interfaces.motor;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import org.ciprix.interfaces.motor.basic.BasicMotorBase;
import org.ciprix.interfaces.motor.basic.BasicMotorConfig;
import org.ciprix.util.RobotClock;

/**
 * Victor SPX (Phoenix 5), open loop only: it has no encoder port and does not measure current.
 *
 * <p>Voltage compensation is enabled at the nominal voltage, so a percent output of 1.0 always
 * means the nominal voltage.
 */
public class MotorIOVictorSPX extends BasicMotorBase {

  /** A controller that does not answer for this long counts as disconnected. */
  public static final double DISCONNECT_TIMEOUT_SECONDS = 0.5;

  private final VictorSPX motor;
  private final Faults faults = new Faults();

  private double lastAnswerTime = Double.NEGATIVE_INFINITY;
  private int lastFaultBits = 0;
  private String[] activeFaults = new String[0];

  public MotorIOVictorSPX(String name, int id, BasicMotorConfig config) {
    super(name, config);
    this.motor = new VictorSPX(id);

    VictorSPXConfiguration spxConfig = new VictorSPXConfiguration();
    spxConfig.peakOutputForward = config.maxOutput;
    spxConfig.peakOutputReverse = config.minOutput;
    motor.configAllSettings(spxConfig);

    motor.configVoltageCompSaturation(nominalVolts);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(Math.max(0.0, config.openLoopRampSeconds));
    motor.setInverted(config.inverted);
    motor.setNeutralMode(config.brakeMode ? NeutralMode.Brake : NeutralMode.Coast);

    if (config.leaderMotorID != 0) {
      motor.set(ControlMode.Follower, config.leaderMotorID);
      motor.setInverted(
          config.followerInverted ? InvertType.OpposeMaster : InvertType.FollowMaster);
    }
  }

  @Override
  protected void updateHardwareInputs(BasicMotorIOInputs inputs) {
    inputs.percentOutput = motor.getMotorOutputPercent();
    inputs.appliedVolts = Volts.of(motor.getMotorOutputVoltage());
    inputs.temperature = Celsius.of(motor.getTemperature());
    boolean ok = motor.getLastError() == ErrorCode.OK;

    motor.getFaults(faults);
    int bits = Phoenix5MotorFaults.problemBits(faults);
    if (bits != lastFaultBits) {
      lastFaultBits = bits;
      activeFaults = Phoenix5MotorFaults.describeProblems(bits);
    }
    inputs.activeFaults = activeFaults;

    double now = RobotClock.now();
    if (ok) {
      lastAnswerTime = now;
    }
    inputs.isConnected = now - lastAnswerTime < DISCONNECT_TIMEOUT_SECONDS;
  }

  @Override
  protected void applyVoltage(double volts) {
    motor.set(ControlMode.PercentOutput, volts / nominalVolts);
  }

  @Override
  protected void applyPercentOutput(double percent) {
    motor.set(ControlMode.PercentOutput, percent);
  }

  @Override
  protected void applyStop() {
    motor.neutralOutput();
  }

  @Override
  protected void applyBrakeMode(boolean enabled) {
    motor.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
  }

  @Override
  protected void applyHardwareOutputRange(double min, double max) {
    super.applyHardwareOutputRange(min, max);
    motor.configPeakOutputForward(max);
    motor.configPeakOutputReverse(min);
  }
}
