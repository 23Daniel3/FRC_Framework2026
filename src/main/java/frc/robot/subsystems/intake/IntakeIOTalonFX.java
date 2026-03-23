package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.interfaces.motor.MotorConfig;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOSparkMax;
import frc.lib.interfaces.motor.MotorIOTalonFX;
import frc.lib.util.CANType;

public class IntakeIOTalonFX implements IntakeIO {

  private final MotorIO rollerMotor;
  private final MotorIO intakeMotor;
  private final DigitalInput coastButton = new DigitalInput(IntakeConstants.SENSOR_PORT);

  public IntakeIOTalonFX() {
    MotorConfig configRollerMotor =
        new MotorConfig()
            .currentLimit(Amps.of(IntakeConstants.CURRENT_LIMIT_ROLLER_MOTOR))
            .coastMode()
            .nominalVoltage(Volts.of(IntakeConstants.NOMINAL_VOLTAGE_ROLLER_MOTOR))
            .inverted(true);

    MotorConfig configIntakeMotor =
        new MotorConfig()
            .currentLimit(Amps.of(IntakeConstants.CURRENT_LIMIT_INTAKE_MOTOR))
            .brakeMode()
            .nominalVoltage(Volts.of(IntakeConstants.VOLTAGE_COMPENSATION_INTAKE_MOTOR))
            .inverted(false);

    rollerMotor =
        new MotorIOTalonFX(IntakeConstants.ROLLER_MOTOR_ID, CANType.RIO, configRollerMotor);
    intakeMotor =
        new MotorIOSparkMax(
            IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless, configIntakeMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerMotorInputs = rollerMotor.getMotorIOInputs();
    inputs.intakeMotorInputs = intakeMotor.getMotorIOInputs();
    inputs.coastButtonPressed = !coastButton.get();
  }

  @Override
  public void resetPosition(Angle position) {
    intakeMotor.setOffset(position);
  }

  @Override
  public void runVelocityRollerMotor(AngularVelocity velocity) {
    rollerMotor.runVelocity(velocity);
  }

  @Override
  public void runPositionIntakeMotor(Angle position) {
    intakeMotor.runPosition(position);
  }

  @Override
  public void runPercentOutputRollerMotor(double percentOutput) {
    rollerMotor.setPercentOutput(percentOutput);
  }

  @Override
  public void runPercentOutputIntakeMotor(double percentOutput) {
    intakeMotor.setPercentOutput(percentOutput);
  }

  @Override
  public void stopRollerMotor() {
    rollerMotor.stop();
  }

  @Override
  public void stopIntakeMotor() {
    intakeMotor.stop();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    intakeMotor.setBrakeMode(enable);
  }

  @Override
  public void configurePIDSVRollerMotor(double kP, double kI, double kD, double kS, double kV) {
    rollerMotor.configurePIDF(0, kP, kI, kD, 0);
    rollerMotor.configureKSVA(0, kS, kV, 0);
  }

  @Override
  public void configurePIDFIntakeMotor(double kP, double kI, double kD, double kF) {
    intakeMotor.configurePIDF(0, kP, kI, kD, kF);
  }

  @Override
  public void setCurrentLimitIntakeMotor(Current current) {
    intakeMotor.setCurrentLimit(current);
  }

  @Override
  public void setCurrentLimitRollerMotor(Current current) {
    rollerMotor.setCurrentLimit(current);
  }

  @Override
  public void setVoltageCompensationIntakeMotor(Voltage voltage) {
    intakeMotor.setVoltageCompensation(voltage);
  }
}
