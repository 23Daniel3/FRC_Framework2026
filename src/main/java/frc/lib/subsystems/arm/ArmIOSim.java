package frc.lib.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

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

public class ArmIOSim implements ArmIO {
  private final LinearSystem<N2, N1, N2> plant =
      LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), 0.025, 50.0);

  private final LinearSystemSim<N2, N1, N2> sim = new LinearSystemSim<>(plant);

  private final PIDController pid = new PIDController(55, ArmConstants.ARM_kI, ArmConstants.ARM_kD);

  private final ArmFeedforward ff = new ArmFeedforward(0.2, 5, 1);

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sim.update(0.02);

    inputs.motorInputs.position = Rotations.of(sim.getOutput(0) / (2 * Math.PI));
    inputs.motorInputs.velocity = RadiansPerSecond.of(sim.getOutput(1));
    inputs.motorInputs.appliedVolts = Volts.of(sim.getInput(0));
    inputs.motorInputs.current = Amps.of(0.0);
  }

  @Override
  public void setVoltage(Voltage volts) {
    sim.setInput(volts.in(Volts) / RobotController.getBatteryVoltage());
  }

  @Override
  public void runPosition(Angle position) {
    double measurement = sim.getOutput(0);
    double pidOutput = pid.calculate(measurement, position.in(Radians));

    double ffVolts = ff.calculate(position.in(Radians), 0.0);
    double volts = pidOutput + ffVolts;

    setVoltage(Volts.of(volts));
  }

  @Override
  public void stop() {
    setVoltage(Volts.of(0.0));
  }
}
