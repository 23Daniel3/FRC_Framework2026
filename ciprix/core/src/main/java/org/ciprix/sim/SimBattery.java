package org.ciprix.sim;

import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import org.ciprix.util.RobotLoop;
import org.littletonrobotics.junction.Logger;

/**
 * The battery of the simulated robot: its voltage drops when the mechanisms draw current, exactly
 * what makes a real robot brown out.
 *
 * <p>Every simulated motor of the library registers its current here, so pushing the robot hard in
 * simulation lowers the voltage that all motors get and the value read by {@code
 * RobotController.getBatteryVoltage()}. Robot code can add other loads (the drivetrain, for
 * example) with {@link #addLoad(DoubleSupplier)}.
 *
 * <p>The model is the usual one: {@code voltage = openCircuit - resistance * totalCurrent}.
 */
public final class SimBattery {

  private static final List<DoubleSupplier> loads = new ArrayList<>();
  private static double openCircuitVolts = 12.5;
  private static double resistanceOhms = 0.02;
  private static double voltage = openCircuitVolts;
  private static double totalCurrent = 0.0;
  private static boolean started = false;

  // Where the voltage goes every cycle. A lambda (not a method reference) so the HAL classes are
  // only touched when it runs, which unit tests without the HAL avoid with setRoboRioSink.
  private static final DoubleConsumer ROBORIO = volts -> RoboRioSim.setVInVoltage(volts);
  private static DoubleConsumer roboRioSink = ROBORIO;

  private SimBattery() {}

  /**
   * Changes the battery model.
   *
   * @param openCircuit voltage with no load (a charged FRC battery is around 12.5 V)
   * @param resistance internal resistance, in ohms (around 0.02 for a good battery)
   */
  public static synchronized void configure(double openCircuit, double resistance) {
    openCircuitVolts = openCircuit;
    resistanceOhms = Math.max(0.0, resistance);
    voltage = openCircuit;
  }

  /**
   * Adds a load to the battery.
   *
   * @param supplyCurrentAmps current drawn from the battery right now, in amps
   */
  public static synchronized void addLoad(DoubleSupplier supplyCurrentAmps) {
    loads.add(supplyCurrentAmps);
    if (!started) {
      started = true;
      RobotLoop.addPeriodic("SimBattery", SimBattery::update);
    }
  }

  /** Battery voltage computed in the last cycle. */
  public static double getVoltage() {
    return voltage;
  }

  /** Total current drawn in the last cycle, in amps. */
  public static double getTotalCurrentAmps() {
    return totalCurrent;
  }

  /** Recomputes the voltage from the registered loads. Pure math, safe in unit tests. */
  public static synchronized double calculate() {
    double sum = 0.0;
    for (int i = 0; i < loads.size(); i++) {
      sum += Math.max(0.0, loads.get(i).getAsDouble());
    }
    totalCurrent = sum;
    voltage = Math.max(0.0, openCircuitVolts - resistanceOhms * sum);
    return voltage;
  }

  /** Runs once per cycle: recomputes the voltage and hands it to the simulated roboRIO. */
  private static void update() {
    calculate();
    roboRioSink.accept(voltage);
    Logger.recordOutput("Sim/Battery/Voltage", voltage);
    Logger.recordOutput("Sim/Battery/CurrentAmps", totalCurrent);
  }

  /**
   * Replaces the simulated roboRIO as the receiver of the voltage. Intended for unit tests that run
   * without the HAL; {@link #resetRoboRioSink()} restores it.
   */
  public static synchronized void setRoboRioSink(DoubleConsumer sink) {
    roboRioSink = sink == null ? volts -> {} : sink;
  }

  /** Sends the voltage to the simulated roboRIO again. */
  public static synchronized void resetRoboRioSink() {
    roboRioSink = ROBORIO;
  }

  /** Forgets every load and restores the default battery (intended for tests). */
  public static synchronized void resetForTesting() {
    loads.clear();
    openCircuitVolts = 12.5;
    resistanceOhms = 0.02;
    voltage = openCircuitVolts;
    totalCurrent = 0.0;
    started = false;
  }
}
