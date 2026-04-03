package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.interfaces.motor.MotorController;
import frc.lib.interfaces.motor.MotorIO;
import frc.lib.interfaces.motor.MotorIOSparkMax;
import frc.lib.interfaces.motor.MotorIOTalonFX;

public class IntakeIOTalonFX implements IntakeIO {

  private final MotorIO rollerMotor;
  private final MotorIO intakeMotor;
  private final DigitalInput coastButton = new DigitalInput(IntakeConstants.SENSOR_PORT);

  public IntakeIOTalonFX() {
    rollerMotor =
        new MotorIOTalonFX(
            "Intake/Roller",
            IntakeConstants.ROLLER_MOTOR_ID,
            new CANBus(),
            IntakeConstants.configRollerMotor);

    intakeMotor =
        new MotorIOSparkMax(
            "Intake/Intake",
            IntakeConstants.INTAKE_MOTOR_ID,
            MotorType.kBrushless,
            IntakeConstants.configIntakeMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    rollerMotor.updateInputs(inputs.rollerMotorInputs);
    intakeMotor.updateInputs(inputs.intakeMotorInputs);

    inputs.coastButtonPressed = !coastButton.get();
  }

  @Override
  public MotorController controlIntakeMotor() {
    return intakeMotor.getMotorController();
  }

  @Override
  public MotorController controlRollerMotor() {
    return rollerMotor.getMotorController();
  }
}
