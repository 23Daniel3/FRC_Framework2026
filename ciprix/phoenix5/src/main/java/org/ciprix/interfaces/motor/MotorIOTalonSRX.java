package org.ciprix.interfaces.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.units.measure.Angle;
import org.ciprix.interfaces.motor.advanced.MotorBase;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.interfaces.motor.basic.BasicMotorConfig;
import org.ciprix.util.Report;
import org.ciprix.util.RobotClock;

/**
 * Talon SRX (Phoenix 5), with the encoder connected to its data port.
 *
 * <p>Units: positions in rotations ({@code ticks / countsPerRevolution x positionConversionFactor})
 * and velocities in rotations per second. The gains are written in Phoenix 5 native units, exactly
 * as given: {@code kP}, {@code kI}, {@code kD} as the Talon SRX expects them, and {@code kV} as
 * {@code kF}. The Talon SRX has no kS, kA or kG; they are reported when set.
 *
 * <p>Voltage compensation is enabled at the nominal voltage, so a percent output of 1.0 always
 * means the nominal voltage.
 */
public class MotorIOTalonSRX extends MotorBase {

  /** A controller that does not answer for this long counts as disconnected. */
  public static final double DISCONNECT_TIMEOUT_SECONDS = 0.5;

  private final TalonSRX motor;
  private final double ticksPerRotation;
  private final Faults faults = new Faults();

  private final double[] cruiseRps = new double[SLOT_COUNT];
  private final double[] accelerationRps2 = new double[SLOT_COUNT];
  private int selectedSlot = 0;
  private int appliedMotionSlot = 0;

  private double lastAnswerTime = Double.NEGATIVE_INFINITY;
  private int lastFaultBits = 0;
  private String[] activeFaults = new String[0];

  public MotorIOTalonSRX(String name, int id, BasicMotorConfig config) {
    this(name, id, MotorConfig.fromBasic(config));
  }

  public MotorIOTalonSRX(String name, int id, MotorConfig config) {
    super(name, config);
    this.motor = new TalonSRX(id);
    double factor = config.positionConversionFactor != 0.0 ? config.positionConversionFactor : 1.0;
    this.ticksPerRotation = config.countsPerRevolution / factor;

    TalonSRXConfiguration srxConfig = new TalonSRXConfiguration();
    srxConfig.peakOutputForward = config.maxOutput;
    srxConfig.peakOutputReverse = config.minOutput;
    if (config.softLimitEnabled) {
      srxConfig.forwardSoftLimitEnable = true;
      srxConfig.forwardSoftLimitThreshold = config.maxPosition.in(Rotations) * ticksPerRotation;
      srxConfig.reverseSoftLimitEnable = true;
      srxConfig.reverseSoftLimitThreshold = config.minPosition.in(Rotations) * ticksPerRotation;
    }
    for (int i = 0; i < SLOT_COUNT; i++) {
      cruiseRps[i] = config.maxMotionMaxVelocity[i].in(RotationsPerSecond);
      accelerationRps2[i] = config.maxMotionMaxAcceleration[i].in(RotationsPerSecondPerSecond);
    }
    srxConfig.motionCruiseVelocity = toNativeVelocity(cruiseRps[0]);
    srxConfig.motionAcceleration = toNativeVelocity(accelerationRps2[0]);
    motor.configAllSettings(srxConfig);

    motor.configVoltageCompSaturation(nominalVolts);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(Math.max(0.0, config.openLoopRampSeconds));
    motor.configClosedloopRamp(Math.max(0.0, config.closedLoopRampSeconds));
    motor.setInverted(config.inverted);
    motor.setNeutralMode(config.brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
    double limit = Math.abs(config.currentLimit.in(Amps));
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, limit, limit, 0));

    boolean feedforwardIgnored = false;
    for (int i = 0; i < SLOT_COUNT; i++) {
      motor.config_kP(i, config.kP[i]);
      motor.config_kI(i, config.kI[i]);
      motor.config_kD(i, config.kD[i]);
      motor.config_kF(i, config.kV[i]);
      feedforwardIgnored |= config.kS[i] != 0 || config.kA[i] != 0 || config.kG[i] != 0;
    }
    if (feedforwardIgnored) {
      Report.warning("Motor " + name + ": the Talon SRX has no kS, kA or kG; they are ignored.");
    }

    if (config.leaderMotorID != 0) {
      motor.set(TalonSRXControlMode.Follower, config.leaderMotorID);
      motor.setInverted(
          config.followerInverted ? InvertType.OpposeMaster : InvertType.FollowMaster);
    }
    motor.setSelectedSensorPosition(0.0);
  }

  private double toNativeVelocity(double rotationsPerSecond) {
    // Phoenix 5 velocity unit: ticks per 100 ms.
    return rotationsPerSecond * ticksPerRotation / 10.0;
  }

  // ---------------------------------------------------------------------------
  // Inputs
  // ---------------------------------------------------------------------------

  @Override
  protected void updateHardwareInputs(BasicMotorIOInputs inputs) {
    inputs.percentOutput = motor.getMotorOutputPercent();
    inputs.appliedVolts = Volts.of(motor.getMotorOutputVoltage());
    inputs.current = Amps.of(motor.getStatorCurrent());
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
  protected void updateHardwareInputs(MotorIOInputs inputs) {
    updateHardwareInputs((BasicMotorIOInputs) inputs);
    inputs.position = Rotations.of(motor.getSelectedSensorPosition() / ticksPerRotation);
    inputs.velocity =
        RotationsPerSecond.of(motor.getSelectedSensorVelocity() * 10.0 / ticksPerRotation);
  }

  // ---------------------------------------------------------------------------
  // Hardware hooks
  // ---------------------------------------------------------------------------

  private void selectSlot(int slot) {
    if (slot != selectedSlot) {
      selectedSlot = slot;
      motor.selectProfileSlot(slot, 0);
    }
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
  protected void applyVelocity(double rotationsPerSecond, int slot) {
    selectSlot(slot);
    motor.set(ControlMode.Velocity, toNativeVelocity(rotationsPerSecond));
  }

  @Override
  protected void applyPosition(double rotations, int slot) {
    selectSlot(slot);
    motor.set(ControlMode.Position, rotations * ticksPerRotation);
  }

  @Override
  protected void applySmartPosition(double rotations, int slot) {
    selectSlot(slot);
    if (slot != appliedMotionSlot) {
      appliedMotionSlot = slot;
      writeMotionLimits();
    }
    motor.set(ControlMode.MotionMagic, rotations * ticksPerRotation);
  }

  private void writeMotionLimits() {
    motor.configMotionCruiseVelocity(toNativeVelocity(cruiseRps[appliedMotionSlot]));
    motor.configMotionAcceleration(toNativeVelocity(accelerationRps2[appliedMotionSlot]));
  }

  @Override
  protected void applyBrakeMode(boolean enabled) {
    motor.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
  }

  @Override
  protected void applyCurrentLimit(double amps) {
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, amps, amps, 0));
  }

  @Override
  public void setOffset(Angle offset) {
    motor.setSelectedSensorPosition(offset.in(Rotations) * ticksPerRotation);
  }

  // ---------------------------------------------------------------------------
  // Tuning (dashboard)
  // ---------------------------------------------------------------------------

  @Override
  public void applyHardwarePID(int slot, double p, double i, double d) {
    motor.config_kP(slot, p);
    motor.config_kI(slot, i);
    motor.config_kD(slot, d);
  }

  @Override
  public void applyHardwareSVAG(int slot, double s, double v, double a, double g) {
    motor.config_kF(slot, v);
  }

  @Override
  public void applyHardwareSmartMotion(
      int slot, double maxVel, double maxAccel, double allowedErr) {
    cruiseRps[slot] = maxVel;
    accelerationRps2[slot] = maxAccel;
    motor.configAllowableClosedloopError(slot, allowedErr * ticksPerRotation);
    if (slot == appliedMotionSlot) {
      writeMotionLimits();
    }
  }

  @Override
  public void applyHardwareOutputRange(int slot, double min, double max) {
    // Peak outputs belong to the controller: the same value arrives for every slot.
    motor.configPeakOutputForward(max);
    motor.configPeakOutputReverse(min);
  }
}
