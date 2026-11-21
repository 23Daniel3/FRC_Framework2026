package org.ciprix.interfaces.motor.impl;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import org.ciprix.CiprixSettings;
import org.ciprix.interfaces.encoder.EncoderIO;
import org.ciprix.interfaces.encoder.EncoderIO.EncoderIOInputs;
import org.ciprix.interfaces.motor.MotorControlMode;
import org.ciprix.interfaces.motor.SoftwareClosedLoop;
import org.ciprix.interfaces.motor.advanced.MotorBase;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.interfaces.motor.basic.BasicMotorIO;
import org.ciprix.util.Report;
import org.ciprix.util.RobotClock;
import org.ciprix.util.RobotStatus;

/**
 * A software-composed {@link org.ciprix.interfaces.motor.advanced.MotorIO} that pairs any {@link
 * BasicMotorIO} for power output with any {@link EncoderIO} for feedback and closes the loop on the
 * roboRIO (PID, feedforward and motion profile from {@link SoftwareClosedLoop}).
 *
 * <p>This is the automatic fallback used by the vendor factories ({@code CtreMotorIOFactory},
 * {@code RevMotorIOFactory}) when no native hardware fusion path exists between the chosen motor
 * controller and the external encoder (e.g., a CIM + a quadrature encoder). It is a full drop-in
 * replacement for any hardware-closed-loop MotorIO.
 *
 * <p>The loop runs inside {@code updateInputs}, right after the encoder is read, so the output is
 * always computed from the newest measurement. The gains follow the WPILib convention: volts per
 * rotation and per rotation per second.
 *
 * <p>Safety: when the encoder stops answering, a closed loop cannot know where the mechanism is, so
 * the motor is stopped (and reported) until the encoder comes back.
 *
 * <p>The power stage must be created with the full output range (see {@code
 * BasicMotorConfig.copyAsPowerStage()}): the output range of {@code config} is applied here.
 */
public class MotorIOComposed extends MotorBase {

  private final BasicMotorIO powerOut;
  private final EncoderIO feedback;
  private final SoftwareClosedLoop loop;

  /** Latest encoder readings refreshed inside updateHardwareInputs. */
  private final EncoderIOInputs feedbackInputs = new EncoderIOInputs();

  private MotorControlMode loopMode = MotorControlMode.IDLE;
  private double loopTarget = 0.0;
  private int loopSlot = 0;
  private double lastLoopTime = Double.NaN;
  private boolean feedbackLossReported = false;

  public MotorIOComposed(
      String name, BasicMotorIO powerOut, EncoderIO feedback, MotorConfig config) {
    super(name, config);
    this.powerOut = powerOut;
    this.feedback = feedback;
    this.loop = new SoftwareClosedLoop(config, SoftwareClosedLoop.Style.VOLTS);
  }

  // ---------------------------------------------------------------------------
  // Inputs and control loop
  // ---------------------------------------------------------------------------

  @Override
  protected void updateHardwareInputs(BasicMotorIOInputs inputs) {
    powerOut.updateInputs(inputs);
  }

  @Override
  protected void updateHardwareInputs(MotorIOInputs inputs) {
    updateHardwareInputs((BasicMotorIOInputs) inputs);
    feedback.updateInputs(feedbackInputs);
    inputs.position = feedbackInputs.position;
    inputs.velocity = feedbackInputs.velocity;
    inputs.isConnected = inputs.isConnected && feedbackInputs.isConnected;
    runLoop();
  }

  private void runLoop() {
    double now = RobotClock.now();
    double dt =
        Double.isNaN(lastLoopTime)
            ? CiprixSettings.getLoopPeriodSeconds()
            : Math.max(1e-3, Math.min(0.1, now - lastLoopTime));
    lastLoopTime = now;

    double position = feedbackInputs.position.in(Rotations);
    double velocity = feedbackInputs.velocity.in(RotationsPerSecond);

    boolean closedLoop =
        loopMode == MotorControlMode.VELOCITY
            || loopMode == MotorControlMode.POSITION
            || loopMode == MotorControlMode.SMART_POSITION;

    if (!closedLoop || RobotStatus.isDisabled() || isOutputOverridden()) {
      // Nothing accumulates while the loop is not in charge (no windup when it comes back).
      loop.reset(position, velocity);
      return;
    }

    if (!feedbackInputs.isConnected) {
      loop.reset(position, velocity);
      powerOut.stop();
      if (!feedbackLossReported) {
        feedbackLossReported = true;
        Report.error(
            "Motor " + name + ": encoder lost, closed loop stopped until the encoder is back.");
      }
      return;
    }
    feedbackLossReported = false;

    double volts = loop.calculate(loopMode, loopTarget, loopSlot, position, velocity, dt);
    powerOut.runVoltage(Volts.of(volts));
  }

  // ---------------------------------------------------------------------------
  // Commands
  // ---------------------------------------------------------------------------

  private void setLoop(MotorControlMode mode, double target, int slot) {
    loopMode = mode;
    loopTarget = target;
    loopSlot = slot;
  }

  @Override
  protected void applyVelocity(double rotationsPerSecond, int slot) {
    setLoop(MotorControlMode.VELOCITY, rotationsPerSecond, slot);
  }

  @Override
  protected void applyPosition(double rotations, int slot) {
    setLoop(MotorControlMode.POSITION, rotations, slot);
  }

  @Override
  protected void applySmartPosition(double rotations, int slot) {
    setLoop(MotorControlMode.SMART_POSITION, rotations, slot);
  }

  @Override
  protected void applyVoltage(double volts) {
    setLoop(MotorControlMode.VOLTAGE, volts, 0);
    powerOut.runVoltage(Volts.of(volts));
  }

  @Override
  protected void applyPercentOutput(double percent) {
    setLoop(MotorControlMode.PERCENT, percent, 0);
    powerOut.runPercentOutput(percent);
  }

  @Override
  protected void applyStop() {
    setLoop(MotorControlMode.IDLE, 0.0, 0);
    powerOut.stop();
  }

  @Override
  protected void applyBrakeMode(boolean enabled) {
    powerOut.setBrakeMode(enabled);
  }

  @Override
  protected void applyCurrentLimit(double amps) {
    powerOut.setCurrentLimit(Amps.of(amps));
  }

  @Override
  public void setOffset(Angle offset) {
    feedback.setPosition(offset);
  }

  // ---------------------------------------------------------------------------
  // Tuning
  // ---------------------------------------------------------------------------

  @Override
  public void applyHardwarePID(int slot, double p, double i, double d) {
    loop.setPID(slot, p, i, d);
  }

  @Override
  public void applyHardwareSVAG(int slot, double s, double v, double a, double g) {
    loop.setFeedforward(slot, s, v, a, g);
  }

  @Override
  public void applyHardwareSmartMotion(
      int slot, double maxVel, double maxAccel, double allowedErr) {
    loop.setMotion(slot, maxVel, maxAccel);
  }

  @Override
  public void applyHardwareOutputRange(int slot, double min, double max) {
    loop.setOutputRange(min, max);
  }
}
