package org.ciprix.interfaces.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import org.ciprix.CiprixSettings;
import org.ciprix.interfaces.motor.advanced.MotorBase;
import org.ciprix.interfaces.motor.advanced.MotorConfig;
import org.ciprix.interfaces.motor.basic.BasicMotorConfig;
import org.ciprix.sim.SimBattery;
import org.ciprix.sim.SimMechanism;
import org.ciprix.util.RobotStatus;

/**
 * A simulated motor: the controller (closed loop, current limit, brake/coast, soft limits) plus the
 * physics of the mechanism it drives.
 *
 * <p>It is built from the <b>same {@link MotorConfig}</b> used by the real motor, and behaves like
 * the real controller with those gains, so what is tuned in simulation is a good starting point for
 * the robot and the state machines can be tested without hardware:
 *
 * <pre>{@code
 * // Real robot
 * roller = new MotorIOTalonFX("Intake/Roller", 18, new CANBus(), IntakeConstants.ROLLER_CONFIG);
 *
 * // Simulation: same config, plus what the mechanism is made of
 * roller = MotorIOSim.talonFX(
 *     "Intake/Roller",
 *     IntakeConstants.ROLLER_CONFIG,
 *     SimMechanism.roller(DCMotor.getKrakenX60(1), 3.0, 0.001));
 * }</pre>
 *
 * <p>Like the real controllers, the loop runs 1000 times per second inside each robot cycle, the
 * reported position starts at zero and uses the conversion factors of the config, and the motor
 * goes to neutral while the robot is disabled. Its current is added to the {@link SimBattery}.
 *
 * <p>For tests, a fault can be injected with {@link #setConnected(boolean)}, {@link
 * #setFaults(String...)} and {@link #setTemperatureCelsius(double)}, and a jam with {@link
 * SimMechanism#withLoad}.
 */
public class MotorIOSim extends MotorBase {

  /** Which real controller the simulated one imitates (it decides what the gains mean). */
  public enum ControllerStyle {
    /** Talon FX / Talon FXS (Phoenix 6): gains in volts per rotation and per rotation/second. */
    TALON_FX,
    /** Spark MAX / Spark Flex: PID in duty cycle per RPM, feedforward in volts per RPM. */
    SPARK
  }

  /** The real controllers close the loop at 1 kHz. */
  private static final double CONTROL_PERIOD_SECONDS = 0.001;

  private static final String[] NO_FAULTS = new String[0];

  private final SimMechanism mechanism;
  private final SoftwareClosedLoop loop;
  private final MotorConfig simConfig;
  private final double positionFactor;
  private final double velocityFactor;

  // What the simulated controller was told to do.
  private MotorControlMode simMode = MotorControlMode.IDLE;
  private double simValue = 0.0;
  private int simSlot = 0;

  private double sensorZero;
  private int controllerCount = 1;
  private boolean connected = true;
  private String[] faults = NO_FAULTS;
  private double temperatureCelsius = 30.0;

  /**
   * Simulated motor driving a mechanism that only has inertia (a roller, a flywheel). The style of
   * the gains is chosen from the motor model: REV motors (NEO, NEO 550, NEO Vortex) behave like a
   * Spark, everything else like a Talon FX.
   *
   * @param name Name for logging
   * @param config Initial configuration
   * @param motor Motor model (e.g., DCMotor.getNeoVortex(1))
   * @param gearing Gear reduction (e.g., 10.0 for 10:1)
   * @param jInertia Moment of inertia in kg*m^2 (e.g., 0.005 for a light intake)
   */
  public MotorIOSim(
      String name, BasicMotorConfig config, DCMotor motor, double gearing, double jInertia) {
    this(name, MotorConfig.fromBasic(config), motor, gearing, jInertia);
  }

  public MotorIOSim(
      String name, MotorConfig config, DCMotor motor, double gearing, double jInertia) {
    this(name, config, SimMechanism.roller(motor, gearing, jInertia), guessStyle(motor));
  }

  /**
   * @param name name for logging
   * @param config the same configuration used by the real motor
   * @param mechanism what the motor drives (see {@link SimMechanism})
   * @param style which controller to imitate
   */
  public MotorIOSim(
      String name, MotorConfig config, SimMechanism mechanism, ControllerStyle style) {
    super(name, config);
    this.simConfig = config;
    this.mechanism = mechanism;
    this.loop =
        new SoftwareClosedLoop(
            config,
            style == ControllerStyle.SPARK
                ? SoftwareClosedLoop.Style.SPARK
                : SoftwareClosedLoop.Style.VOLTS);

    double position =
        config.positionConversionFactor != 0.0 ? config.positionConversionFactor : 1.0;
    double velocity =
        config.velocityConversionFactor != 0.0 ? config.velocityConversionFactor : 1.0;
    this.positionFactor = position;
    // A Talon scales position and velocity by the same ratio; a Spark has one factor for each.
    this.velocityFactor = style == ControllerStyle.SPARK ? velocity : position;

    // Like the real controllers, the sensor reads zero where the mechanism is at boot.
    this.sensorZero = mechanism.getRotorPositionRotations() * positionFactor;

    SimBattery.addLoad(() -> mechanism.getSupplyCurrentAmps(SimBattery.getVoltage()));
  }

  /** Simulated Talon FX / Talon FXS driving {@code mechanism}. */
  public static MotorIOSim talonFX(String name, MotorConfig config, SimMechanism mechanism) {
    return new MotorIOSim(name, config, mechanism, ControllerStyle.TALON_FX);
  }

  /** Simulated Spark MAX / Spark Flex driving {@code mechanism}. */
  public static MotorIOSim spark(String name, MotorConfig config, SimMechanism mechanism) {
    return new MotorIOSim(name, config, mechanism, ControllerStyle.SPARK);
  }

  private static ControllerStyle guessStyle(DCMotor motor) {
    boolean rev =
        sameModel(motor, DCMotor.getNEO(1))
            || sameModel(motor, DCMotor.getNeoVortex(1))
            || sameModel(motor, DCMotor.getNeo550(1));
    return rev ? ControllerStyle.SPARK : ControllerStyle.TALON_FX;
  }

  /** Kv and Kt do not depend on how many motors share the gearbox, so they identify the model. */
  private static boolean sameModel(DCMotor a, DCMotor b) {
    return Math.abs(a.KvRadPerSecPerVolt - b.KvRadPerSecPerVolt) < 1e-6 * b.KvRadPerSecPerVolt
        && Math.abs(a.KtNMPerAmp - b.KtNMPerAmp) < 1e-6 * b.KtNMPerAmp;
  }

  /**
   * Declares that the mechanism is driven by more than one controller (a leader and its followers),
   * all included in the motor model (e.g. {@code DCMotor.getKrakenX60(2)}). The current limit then
   * applies to each controller and the reported current is the one of a single motor.
   *
   * @param followers number of followers (1 for a leader with one follower)
   */
  public MotorIOSim withFollowers(int followers) {
    this.controllerCount = 1 + Math.max(0, followers);
    return this;
  }

  // ---------------------------------------------------------------------------
  // Simulation
  // ---------------------------------------------------------------------------

  private double sensorPosition() {
    return mechanism.getRotorPositionRotations() * positionFactor - sensorZero;
  }

  private double sensorVelocity() {
    return mechanism.getRotorVelocityRps() * velocityFactor;
  }

  private void simulate() {
    double period = CiprixSettings.getLoopPeriodSeconds();
    int steps = Math.max(1, (int) Math.round(period / CONTROL_PERIOD_SECONDS));
    double dt = period / steps;

    boolean enabled = RobotStatus.isEnabled();
    double battery = SimBattery.getVoltage();
    double currentLimit = getCurrentLimitAmps() * controllerCount;
    boolean softLimits = simConfig.softLimitEnabled;
    double softMin = simConfig.minPosition.in(Rotations);
    double softMax = simConfig.maxPosition.in(Rotations);

    for (int i = 0; i < steps; i++) {
      double position = sensorPosition();
      double velocity = sensorVelocity();

      if (!enabled || simMode == MotorControlMode.IDLE) {
        // A disabled controller is in neutral and its loop does not accumulate anything.
        loop.reset(position, velocity);
        mechanism.stepNeutral(dt, isBrakeMode());
        continue;
      }

      double volts;
      switch (simMode) {
        case VOLTAGE -> {
          loop.reset(position, velocity);
          volts = simValue;
        }
        case PERCENT -> {
          loop.reset(position, velocity);
          volts = simValue * nominalVolts;
        }
        default -> volts = loop.calculate(simMode, simValue, simSlot, position, velocity, dt);
      }

      if (softLimits
          && ((volts > 0.0 && position >= softMax) || (volts < 0.0 && position <= softMin))) {
        mechanism.stepNeutral(dt, isBrakeMode());
      } else {
        mechanism.step(volts, dt, currentLimit, battery);
      }
    }
  }

  @Override
  protected void updateHardwareInputs(BasicMotorIOInputs inputs) {
    simulate();
    double battery = SimBattery.getVoltage();
    double applied = mechanism.getAppliedVolts();
    inputs.appliedVolts = Volts.of(applied);
    inputs.percentOutput = battery > 1e-6 ? applied / battery : 0.0;
    inputs.current = Amps.of(Math.abs(mechanism.getCurrentAmps()) / controllerCount);
    inputs.temperature = Celsius.of(temperatureCelsius);
    inputs.isConnected = connected;
    inputs.activeFaults = faults;
  }

  @Override
  protected void updateHardwareInputs(MotorIOInputs inputs) {
    updateHardwareInputs((BasicMotorIOInputs) inputs);
    inputs.position = Rotations.of(sensorPosition());
    inputs.velocity = RotationsPerSecond.of(sensorVelocity());
  }

  /**
   * Fills the inputs of a follower with what its leader (this motor) is doing, for subsystems that
   * log the follower too.
   */
  public void updateFollowerInputs(MotorIOInputs followerInputs) {
    MotorIOInputs leader = getMotorIOInputs();
    followerInputs.appliedVolts = leader.appliedVolts;
    followerInputs.percentOutput = leader.percentOutput;
    followerInputs.current = leader.current;
    followerInputs.temperature = leader.temperature;
    followerInputs.isConnected = leader.isConnected;
    followerInputs.activeFaults = leader.activeFaults;
    followerInputs.position = leader.position;
    followerInputs.velocity = leader.velocity;
    followerInputs.averageCurrentAmps = leader.averageCurrentAmps;
    followerInputs.hasProblem = leader.hasProblem;
    followerInputs.problem = leader.problem;
  }

  // ---------------------------------------------------------------------------
  // The simulated controller receives the commands
  // ---------------------------------------------------------------------------

  private void set(MotorControlMode mode, double value, int slot) {
    simMode = mode;
    simValue = value;
    simSlot = slot;
  }

  @Override
  protected void applyVoltage(double volts) {
    set(MotorControlMode.VOLTAGE, volts, 0);
  }

  @Override
  protected void applyPercentOutput(double percent) {
    set(MotorControlMode.PERCENT, percent, 0);
  }

  @Override
  protected void applyStop() {
    set(MotorControlMode.IDLE, 0.0, 0);
  }

  @Override
  protected void applyVelocity(double rotationsPerSecond, int slot) {
    set(MotorControlMode.VELOCITY, rotationsPerSecond, slot);
  }

  @Override
  protected void applyPosition(double rotations, int slot) {
    set(MotorControlMode.POSITION, rotations, slot);
  }

  @Override
  protected void applySmartPosition(double rotations, int slot) {
    set(MotorControlMode.SMART_POSITION, rotations, slot);
  }

  @Override
  public void setOffset(Angle offset) {
    sensorZero = mechanism.getRotorPositionRotations() * positionFactor - offset.in(Rotations);
  }

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

  // ---------------------------------------------------------------------------
  // Test hooks
  // ---------------------------------------------------------------------------

  /** The physics behind this motor. */
  public SimMechanism getMechanism() {
    return mechanism;
  }

  /** Simulates the controller dropping off (or coming back to) the CAN bus. */
  public void setConnected(boolean connected) {
    this.connected = connected;
  }

  /** Simulates active faults reported by the controller. No arguments clears them. */
  public void setFaults(String... activeFaults) {
    this.faults = activeFaults == null || activeFaults.length == 0 ? NO_FAULTS : activeFaults;
  }

  /** Simulates the temperature reported by the controller. */
  public void setTemperatureCelsius(double celsius) {
    this.temperatureCelsius = celsius;
  }
}
