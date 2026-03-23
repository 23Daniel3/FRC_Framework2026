package frc.lib.subsystems.wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class WristIOSim implements WristIO {

  private final LinearSystem<N2, N1, N2> plant =
      LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), 0.01, 100.0);

  private final LinearSystemSim<N2, N1, N2> sim = new LinearSystemSim<>(plant);

  private final PIDController pid = new PIDController(220.0, 0.0, 0.0);
  private final ArmFeedforward ff = new ArmFeedforward(0.2, 0.5, 0.5);

  private double setpointRad = 0.0;

  @Override
  public void updateInputs(WristIOInputs inputs) {

    sim.update(0.02);

    double positionRad = sim.getOutput(0);
    double velocityRadPerSec = sim.getOutput(1);
    double positionRot = Units.radiansToRotations(positionRad);

    inputs.motorInputs.velocity = RadiansPerSecond.of(velocityRadPerSec);
    inputs.motorInputs.position = Rotations.of(positionRot);
    inputs.motorInputs.appliedVolts = Volts.of(sim.getInput(0));
    inputs.motorInputs.current = Amps.of(0.0);
  }

  @Override
  public void setVoltage(Voltage volts) {
    sim.setInput(volts.in(Volts) / RobotController.getBatteryVoltage());
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
  public void resetEncoder() {
    sim.setState(VecBuilder.fill(0.0, 0.0));
    setpointRad = 0.0;
    pid.reset();
  }

  @Override
  public void runPosition(Angle positionRot) {
    double measurementRad = sim.getOutput(0);
    double pidOutput = pid.calculate(measurementRad, setpointRad);

    double ffVolts = ff.calculate(setpointRad, 0.0);
    double volts = pidOutput + ffVolts;

    setVoltage(Volts.of(volts));
  }

  public double getEncoder() {
    return Units.radiansToRotations(sim.getOutput(0));
  }
}
