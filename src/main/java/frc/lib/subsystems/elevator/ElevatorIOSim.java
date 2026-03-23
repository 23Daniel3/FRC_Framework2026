package frc.lib.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class ElevatorIOSim implements ElevatorIO {
  private final LinearSystem<N2, N1, N2> plant =
      LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(2), 0.025, 50.0);

  private final LinearSystemSim<N2, N1, N2> sim = new LinearSystemSim<>(plant);

  private final PIDController pid = new PIDController(30.0, 0.0, 0.0);
  private final ArmFeedforward ff = new ArmFeedforward(0.2, 5.0, 1.0);

  private double setpointRad = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.update(0.02);

    inputs.motorLeftInputs.position = Radians.of(sim.getOutput(0));
    inputs.motorLeftInputs.velocity = RadiansPerSecond.of(sim.getOutput(1));
    inputs.motorLeftInputs.appliedVolts = Volts.of(sim.getInput(0));
    inputs.motorLeftInputs.current = Amps.of(0.0);

    inputs.motorRightInputs.position = Radians.of(sim.getOutput(0));
    inputs.motorRightInputs.velocity = RadiansPerSecond.of(sim.getOutput(1));
    inputs.motorRightInputs.appliedVolts = Volts.of(sim.getInput(0));
    inputs.motorRightInputs.current = Amps.of(0.0);
  }

  @Override
  public void setVoltage(Voltage volts) {
    sim.setInput(volts.in(Volts) / RobotController.getBatteryVoltage());
  }

  @Override
  public void runPosition(Angle position) {
    this.setpointRad = position.in(Radians);

    double measurementRad = sim.getOutput(0);
    double pidOutput = pid.calculate(measurementRad, setpointRad);

    double ffVolts = ff.calculate(setpointRad, 0.0);
    double volts = pidOutput + ffVolts;

    setVoltage(Volts.of(volts));
  }

  @Override
  public void setPercentOutput(double percentOutput) {
    setVoltage(Volts.of(percentOutput * RobotController.getBatteryVoltage()));
  }

  @Override
  public void stop() {
    setVoltage(Volts.of(0.0));
  }

  @Override
  public void reset() {
    sim.setState(VecBuilder.fill(0.0, 0.0));
    setpointRad = 0.0;
    pid.reset();
  }
}
