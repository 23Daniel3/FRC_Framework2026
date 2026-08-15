package frc.lib.interfaces.motor.basic;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.motor.MotorControlMode;
import frc.lib.logger.LoggedTunableNumber;
import frc.robot.Constants;

/**
 * Shared base for every motor IO in this package — both {@link BasicMotorIO}-only implementations
 * (CIM/RedLine via PWM, Talon SRX, SparkMax brushed...) and, through {@link
 * frc.lib.interfaces.motor.advanced.MotorBase}, every closed-loop-capable implementation
 * (SparkFlex, TalonFX, ...).
 *
 * <p>Everything that is meaningful for a simple open-loop motor lives here: the current control
 * mode, the tunable output-range clamp (dashboard-editable in tuning mode), the software clamp
 * itself, and the {@link BasicMotorController} facade. {@link
 * frc.lib.interfaces.motor.advanced.MotorBase} extends this class and reuses all of it, adding only
 * what closed-loop control needs on top.
 */
public abstract class BasicMotorBase implements BasicMotorIO {

  protected final String name;
  protected MotorControlMode currentMode = MotorControlMode.IDLE;
  protected final boolean tuningMode = Constants.tuningMode;

  private double minOutput;
  private double maxOutput;
  private final TunableLimits limits;

  /**
   * Base implementation of {@link BasicMotorController}, delegating every call back to this IO.
   * {@link frc.lib.interfaces.motor.advanced.MotorBase} extends this inner class instead of
   * rebuilding it, so the basic delegation logic is written exactly once.
   */
  protected class BasicControllerImpl implements BasicMotorController {

    @Override
    public void setBrakeMode(boolean enabled) {
      BasicMotorBase.this.setBrakeMode(enabled);
    }

    @Override
    public void runVoltage(Voltage volts) {
      BasicMotorBase.this.runVoltage(volts);
    }

    @Override
    public void runPercentOutput(double percent) {
      BasicMotorBase.this.runPercentOutput(percent);
    }

    @Override
    public void stop() {
      BasicMotorBase.this.stop();
    }

    @Override
    public void setCurrentLimit(Current current) {
      BasicMotorBase.this.setCurrentLimit(current);
    }
  }

  private final BasicMotorController controller = new BasicControllerImpl();

  public BasicMotorBase(String name, BasicMotorConfig config) {
    this.name = name;
    this.minOutput = config.minOutput;
    this.maxOutput = config.maxOutput;
    this.limits = new TunableLimits(name, config.minOutput, config.maxOutput);
  }

  @Override
  public void updateInputs(BasicMotorIOInputs inputs) {
    checkOutputRangeTuning();
    updateHardwareInputs(inputs);
  }

  /**
   * Clamps a requested percent-output to the configured (and live-tunable) output range. All {@code
   * runPercentOutput} implementations should route through this before touching hardware.
   */
  protected double clampOutput(double percent) {
    return MathUtil.clamp(percent, minOutput, maxOutput);
  }

  /** Re-checks the tunable output-range dashboard entries, applying them if changed. */
  protected void checkOutputRangeTuning() {
    if (tuningMode) {
      limits.check();
    }
  }

  private class TunableLimits {
    private final LoggedTunableNumber min, max;

    TunableLimits(String n, double vmin, double vmax) {
      this.min = new LoggedTunableNumber(n + "/Config/MinOutput", vmin);
      this.max = new LoggedTunableNumber(n + "/Config/MaxOutput", vmax);
    }

    void check() {
      if (min.hasChanged(hashCode()) || max.hasChanged(hashCode())) {
        applyHardwareOutputRange(min.get(), max.get());
      }
    }
  }

  /**
   * Called whenever the tunable output range changes. Default behavior just updates the software
   * clamp used by {@link #clampOutput(double)} — sufficient for controllers with no native
   * output-range concept (PWM, Talon SRX, SparkMax brushed).
   *
   * <p>{@link frc.lib.interfaces.motor.advanced.MotorBase} overrides this to additionally push the
   * range into every closed-loop slot on the hardware.
   */
  protected void applyHardwareOutputRange(double min, double max) {
    this.minOutput = min;
    this.maxOutput = max;
  }

  protected abstract void updateHardwareInputs(BasicMotorIOInputs inputs);

  @Override
  public BasicMotorController getMotorController() {
    return controller;
  }
}
