package frc.lib.interfaces.motor.impl;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.encoder.EncoderIO;
import frc.lib.interfaces.encoder.EncoderIO.EncoderIOInputs;
import frc.lib.interfaces.motor.MotorControlMode;
import frc.lib.interfaces.motor.advanced.MotorBase;
import frc.lib.interfaces.motor.advanced.MotorConfig;
import frc.lib.interfaces.motor.basic.BasicMotorIO;
import frc.lib.util.PeriodicSystem;

/**
 * A software-composed {@link frc.lib.interfaces.motor.advanced.MotorIO} that pairs any {@link
 * BasicMotorIO} for power output with any {@link EncoderIO} for positional feedback and runs a
 * WPILib PID+FF loop on the RIO at 50 Hz via {@link PeriodicSystem}.
 *
 * <p>This is the automatic fallback used by {@link MotorIOFactory} when no native hardware fusion
 * path exists between the chosen motor controller and the external encoder (e.g., a CIM + a
 * quadrature encoder). It is a full drop-in replacement for any hardware-closed-loop MotorIO.
 */
public class MotorIOComposed extends MotorBase {

  private final BasicMotorIO powerOut;
  private final EncoderIO feedback;

  /** Internal inputs struct — same pattern as every hardware MotorIO. */
  private final MotorIOInputs inputs = new MotorIOInputs();

  /** Latest encoder readings refreshed inside updateHardwareInputs. */
  private final EncoderIOInputs feedbackInputs = new EncoderIOInputs();

  // Per-slot software control objects
  private final PIDController[] pidControllers = new PIDController[4];
  private final SimpleMotorFeedforward[] feedforwards = new SimpleMotorFeedforward[4];
  private final TrapezoidProfile[] profiles = new TrapezoidProfile[4];

  private TrapezoidProfile.State profileState = new TrapezoidProfile.State();
  private int activeSlot = 0;

  // Background periodic control loop (registers with the WPILib scheduler automatically)
  @SuppressWarnings("unused")
  private final PeriodicSystem controlLoop;

  public MotorIOComposed(
      String name, BasicMotorIO powerOut, EncoderIO feedback, MotorConfig config) {
    super(name, config);
    this.powerOut = powerOut;
    this.feedback = feedback;

    for (int i = 0; i < 4; i++) {
      pidControllers[i] = new PIDController(config.kP[i], config.kI[i], config.kD[i]);
      feedforwards[i] = new SimpleMotorFeedforward(config.kS[i], config.kV[i], config.kA[i]);
      if (config.maxMotionMaxVelocity[i] != null
          && config.maxMotionMaxVelocity[i].in(RotationsPerSecond) > 0) {
        profiles[i] =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    config.maxMotionMaxVelocity[i].in(RotationsPerSecond),
                    config.maxMotionMaxAcceleration[i].in(RotationsPerSecondPerSecond)));
      }
    }

    this.controlLoop =
        new PeriodicSystem(name + "_SoftwarePID") {
          @Override
          public void periodic() {
            applyControlLoop();
          }
        };
  }

  // ---------------------------------------------------------------------------
  // Hardware-inputs bridge (required by MotorBase)
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
  }

  /** Returns the shared inputs struct used by this instance. */
  @Override
  public MotorIOInputs getMotorIOInputs() {
    return inputs;
  }

  // ---------------------------------------------------------------------------
  // Background control loop
  // ---------------------------------------------------------------------------

  private void applyControlLoop() {
    switch (currentMode) {
      case VELOCITY -> {
        double tgt = targetVelocity.in(RotationsPerSecond);
        double meas = feedbackInputs.velocity.in(RotationsPerSecond);
        double out =
            pidControllers[activeSlot].calculate(meas, tgt)
                + feedforwards[activeSlot].calculate(tgt);
        powerOut.runVoltage(Volts.of(out));
      }
      case POSITION -> {
        double tgt = targetPosition.in(Rotations);
        double meas = feedbackInputs.position.in(Rotations);
        double out =
            pidControllers[activeSlot].calculate(meas, tgt)
                + feedforwards[activeSlot].calculate(0.0);
        powerOut.runVoltage(Volts.of(out));
      }
      case SMART_POSITION -> {
        if (profiles[activeSlot] != null) {
          double tgt = targetPosition.in(Rotations);
          double meas = feedbackInputs.position.in(Rotations);
          TrapezoidProfile.State goal = new TrapezoidProfile.State(tgt, 0.0);
          profileState = profiles[activeSlot].calculate(0.02, profileState, goal);
          double out =
              pidControllers[activeSlot].calculate(meas, profileState.position)
                  + feedforwards[activeSlot].calculate(profileState.velocity);
          powerOut.runVoltage(Volts.of(out));
        }
      }
      case VOLTAGE, PERCENT, IDLE -> {
        // Handled directly by runVoltage / runPercentOutput / stop
      }
    }
  }

  // ---------------------------------------------------------------------------
  // MotorIO — control commands
  // ---------------------------------------------------------------------------

  @Override
  public void runVelocity(AngularVelocity velocity) {
    runVelocity(velocity, 0);
  }

  @Override
  public void runPosition(Angle position) {
    runPosition(position, 0);
  }

  @Override
  public void runSmartPosition(Angle position) {
    runSmartPosition(position, 0);
  }

  @Override
  public void runVelocity(AngularVelocity velocity, int slot) {
    this.currentMode = MotorControlMode.VELOCITY;
    this.targetVelocity = velocity;
    this.activeSlot = slot;
    pidControllers[slot].reset();
  }

  @Override
  public void runPosition(Angle position, int slot) {
    this.currentMode = MotorControlMode.POSITION;
    this.targetPosition = position;
    this.activeSlot = slot;
    pidControllers[slot].reset();
  }

  @Override
  public void runSmartPosition(Angle position, int slot) {
    this.currentMode = MotorControlMode.SMART_POSITION;
    this.targetPosition = position;
    this.activeSlot = slot;
    pidControllers[slot].reset();
    profileState =
        new TrapezoidProfile.State(
            feedbackInputs.position.in(Rotations), feedbackInputs.velocity.in(RotationsPerSecond));
  }

  @Override
  public void runVoltage(Voltage volts) {
    this.currentMode = MotorControlMode.IDLE;
    powerOut.runVoltage(volts);
  }

  @Override
  public void runPercentOutput(double percent) {
    this.currentMode = MotorControlMode.IDLE;
    powerOut.runPercentOutput(percent);
  }

  @Override
  public void stop() {
    this.currentMode = MotorControlMode.IDLE;
    powerOut.stop();
  }

  // ---------------------------------------------------------------------------
  // BasicMotorIO — delegates to the wrapped BasicMotorIO
  // ---------------------------------------------------------------------------

  @Override
  public void setBrakeMode(boolean enabled) {
    powerOut.setBrakeMode(enabled);
  }

  @Override
  public void setCurrentLimit(Current current) {
    powerOut.setCurrentLimit(current);
  }

  @Override
  public void setOffset(Angle offset) {
    feedback.setPosition(offset);
  }

  // ---------------------------------------------------------------------------
  // Tuning — wire into WPILib PID objects
  // ---------------------------------------------------------------------------

  @Override
  public void applyHardwarePID(int slot, double p, double i, double d) {
    pidControllers[slot].setPID(p, i, d);
  }

  @Override
  public void applyHardwareSVAG(int slot, double s, double v, double a, double g) {
    feedforwards[slot] = new SimpleMotorFeedforward(s, v, a);
  }

  @Override
  public void applyHardwareSmartMotion(
      int slot, double maxVel, double maxAccel, double allowedErr) {
    profiles[slot] = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVel, maxAccel));
  }

  @Override
  public void applyHardwareOutputRange(int slot, double min, double max) {
    // Output clamping could be added here if required
  }
}
