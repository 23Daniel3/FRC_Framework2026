package frc.lib.subsystems.funnel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class FunnelIOSim implements FunnelIO {
  private static final DCMotor MOTOR = DCMotor.getNeo550(1);
  private static final double GEAR_RATIO = 10.0;
  private static final double J_KG_M2 = 0.0005;
  private static final double SIM_LOOP_PERIOD = 0.02;
  private static final double FRICTION_COEFF = 0.02;
  private static final double SENSOR_NOISE = 0.005;

  private final LinearSystem<N1, N1, N1> plant =
      LinearSystemId.createFlywheelSystem(MOTOR, J_KG_M2, GEAR_RATIO);

  private final LinearSystemSim<N1, N1, N1> sim = new LinearSystemSim<>(plant);

  private double appliedVoltage = 0.0;
  private double positionRad = 0.0;
  private double lastSetpointRadPerSec = 0.0;

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    sim.update(SIM_LOOP_PERIOD);

    double velocity = sim.getOutput(0);

    velocity -= velocity * FRICTION_COEFF;

    positionRad += velocity * SIM_LOOP_PERIOD;

    double noise = (Math.random() - 0.5) * 2.0 * SENSOR_NOISE;

    inputs.motorInputs.position = Rotations.of((positionRad + noise) / (2 * Math.PI));
    inputs.motorInputs.velocity = RadiansPerSecond.of(velocity + noise);
    inputs.motorInputs.appliedVolts = Volts.of(appliedVoltage);
    inputs.motorInputs.current = Amps.of(MOTOR.getCurrent(velocity * GEAR_RATIO, appliedVoltage));
  }

  @Override
  public void setVoltage(Voltage volts) {
    appliedVoltage = clampVoltage(volts.in(Volts));
    sim.setInput(appliedVoltage);
  }

  @Override
  public void runVelocity(AngularVelocity velocity) {
    double kV = 1.0 / MOTOR.KvRadPerSecPerVolt;
    appliedVoltage = clampVoltage(velocity.in(RadiansPerSecond) * kV);
    lastSetpointRadPerSec = velocity.in(RadiansPerSecond);
    sim.setInput(appliedVoltage);
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    sim.setInput(0.0);
    lastSetpointRadPerSec = 0.0;
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {}

  @Override
  public void resetPosition() {
    positionRad = 0.0;
  }

  @Override
  public void setPercentOutput(double percentOutput) {
    setVoltage(Volts.of(percentOutput * RobotController.getBatteryVoltage()));
  }

  private double clampVoltage(double volts) {
    return Math.max(
        -RobotController.getBatteryVoltage(), Math.min(volts, RobotController.getBatteryVoltage()));
  }

  public double getLastSetpointRadPerSec() {
    return lastSetpointRadPerSec;
  }
}
