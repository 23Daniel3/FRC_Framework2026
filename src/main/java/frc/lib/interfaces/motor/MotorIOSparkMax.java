package frc.lib.interfaces.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.security.SparkUtil;

public class MotorIOSparkMax extends MotorBase {

    private final SparkMax motor;
    private final SparkClosedLoopController closedLoopController;
    private final SparkMaxConfig motorConfig;
    private final MotorIOInputs inputs = new MotorIOInputs();

    // Sensores
    private RelativeEncoder internalEncoder;
    private RelativeEncoder externalEncoder;
    private AbsoluteEncoder absoluteEncoder;
    private final MotorConfig.FeedbackSensorType sensorType;

    // Arrays para armazenar parâmetros de SVAG que não rodam nativos na malha do Spark
    private final double[] kS = new double[4];
    private final double[] kA = new double[4];
    private final double[] kG = new double[4];

    public MotorIOSparkMax(String name, int id, MotorType type, MotorConfig config, boolean tuningMode) {
        // Inicializa o MotorBase (os métodos apply... chamados no super irão retornar silenciosamente pois motor == null)
        super(name, tuningMode, config);

        this.motor = new SparkMax(id, type);
        this.closedLoopController = motor.getClosedLoopController();
        this.motorConfig = new SparkMaxConfig();
        this.sensorType = config.feedbackType;

        // --- Configuração Básica ---
        motorConfig.inverted(config.inverted)
                   .smartCurrentLimit((int) config.currentLimit.in(Amps))
                   .idleMode(config.idleMode)
                   .voltageCompensation(config.nominalVoltage.in(Volts));

        // --- Seleção de Encoder ---
        switch (config.feedbackType) {
            case ALTERNATE -> {
                motorConfig.alternateEncoder.countsPerRevolution(config.countsPerRevolution)
                           .positionConversionFactor(config.positionConversionFactor)
                           .velocityConversionFactor(config.velocityConversionFactor);
                motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
                externalEncoder = motor.getAlternateEncoder();
            }
            case ABSOLUTE_DATAPORT -> {
                motorConfig.absoluteEncoder.positionConversionFactor(config.positionConversionFactor)
                           .velocityConversionFactor(config.velocityConversionFactor)
                           .zeroOffset(0.0);
                motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
                absoluteEncoder = motor.getAbsoluteEncoder();
            }
            default -> {
                motorConfig.encoder.positionConversionFactor(config.positionConversionFactor)
                           .velocityConversionFactor(config.velocityConversionFactor);
                motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                internalEncoder = motor.getEncoder();
            }
        }

        // --- Injeção Inicial de PID/SVAG/SmartMotion (Todos os Slots) ---
        for (int i = 0; i < 4; i++) {
            ClosedLoopSlot slot = resolveSlot(i);
            motorConfig.closedLoop.p(config.kP[i], slot)
                                  .i(config.kI[i], slot)
                                  .d(config.kD[i], slot)
                                  .outputRange(config.minOutput, config.maxOutput, slot)
                                  .apply(new FeedForwardConfig().kV(config.kV[i], slot));

            // Salva kS, kA e kG para processamento de Arbitrary FF nas chamadas de controle
            this.kS[i] = config.kS[i];
            this.kA[i] = config.kA[i];
            this.kG[i] = config.kG[i];

            // Configuração do MAXMotion
            if (config.maxMotionMaxVelocity[i].in(RadiansPerSecond) > 0) {
                double rpm = config.maxMotionMaxVelocity[i].in(RotationsPerSecond) * 60.0;
                double rpmPerSec = config.maxMotionMaxAcceleration[i].in(RotationsPerSecondPerSecond) * 60.0;
                motorConfig.closedLoop.maxMotion.cruiseVelocity(rpm, slot)
                                                .maxAcceleration(rpmPerSec, slot)
                                                .allowedProfileError(config.maxMotionAllowedClosedLoopError[i].in(Rotations), slot);
            }
        }

        // --- Soft Limits e Wrapping ---
        if (config.softLimitEnabled) {
            motorConfig.softLimit.forwardSoftLimitEnabled(true)
                                 .forwardSoftLimit(config.maxPosition.in(Rotations))
                                 .reverseSoftLimitEnabled(true)
                                 .reverseSoftLimit(config.minPosition.in(Rotations));
        }

        if (config.positionWrap) {
            motorConfig.closedLoop.positionWrappingEnabled(true)
                                  .positionWrappingInputRange(config.minPosition.in(Rotations), config.maxPosition.in(Rotations));
        }

        // --- Push final das configurações para o Hardware ---
        applyConfig(true);
    }

    @Override
    protected void updateHardwareInputs(MotorIOInputs inputs) {
        switch (sensorType) {
            case ALTERNATE -> {
                if (externalEncoder != null) {
                    inputs.position = Rotations.of(externalEncoder.getPosition());
                    inputs.velocity = RadiansPerSecond.of(Units.rotationsPerMinuteToRadiansPerSecond(externalEncoder.getVelocity()));
                }
            }
            case ABSOLUTE_DATAPORT -> {
                if (absoluteEncoder != null) {
                    inputs.position = Rotations.of(absoluteEncoder.getPosition());
                    inputs.velocity = RadiansPerSecond.of(Units.rotationsPerMinuteToRadiansPerSecond(absoluteEncoder.getVelocity()));
                }
            }
            default -> {
                if (internalEncoder != null) {
                    inputs.position = Rotations.of(internalEncoder.getPosition());
                    inputs.velocity = RadiansPerSecond.of(Units.rotationsPerMinuteToRadiansPerSecond(internalEncoder.getVelocity()));
                }
            }
        }

        inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.current = Amps.of(motor.getOutputCurrent());
        inputs.temperature = Celsius.of(motor.getMotorTemperature());
        inputs.isConnected = !motor.hasActiveFault();
        inputs.activeFaults = motor.hasActiveFault() ? frc.lib.interfaces.motor.MotorFaults.getSparkFaults(motor) : new String[]{};
    }

    // --- Controle de Movimento com Arbitrary FF Injetado ---

    @Override
    public void runVelocity(AngularVelocity velocity) {
        double velocityRPM = velocity.in(RPM);
        // Aplica atrito estático (kS) respeitando a direção do movimento + Gravidade (kG)
        double arbFF = (kS[0] * Math.signum(velocityRPM)) + kG[0];

        closedLoopController.setSetpoint(
            velocityRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0, arbFF, SparkClosedLoopController.ArbFFUnits.kVoltage
        );
    }

    @Override
    public void runPosition(Angle position) {
        // Para posição, injetamos a gravidade (kG) para manter braços/elevadores parados.
        closedLoopController.setSetpoint(
            position.in(Rotations), ControlType.kPosition, ClosedLoopSlot.kSlot0, kG[0], SparkClosedLoopController.ArbFFUnits.kVoltage
        );
    }

    @Override
    public void runSmartPosition(Angle position) {
        closedLoopController.setSetpoint(
            position.in(Rotations), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, kG[0], SparkClosedLoopController.ArbFFUnits.kVoltage
        );
    }

    // --- Controles de Baixo Nível ---

    @Override
    public void runVoltage(Voltage volts) {
        motor.setVoltage(volts.in(Volts));
    }

    @Override
    public void runPercentOutput(Voltage percent) {
        motor.set(percent.in(Volts)); 
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setOffset(Angle offset) {
        if (internalEncoder != null) internalEncoder.setPosition(offset.in(Rotations));
        if (externalEncoder != null) externalEncoder.setPosition(offset.in(Rotations));
    }

    // --- Atualização de Hardware via Dashboard (Tuning) ---

    @Override
    public void setBrakeMode(boolean enabled) {
        if (motor == null) return; // Previne NullPointer no Super
        motorConfig.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
        applyConfig(false);
    }

    @Override
    public void setCurrentLimit(Current current) {
        if (motor == null) return;
        motorConfig.smartCurrentLimit((int) current.in(Amps));
        applyConfig(false);
    }

    @Override
    public void applyHardwarePID(int slot, double p, double i, double d) {
        if (motor == null) return;
        motorConfig.closedLoop.p(p, resolveSlot(slot)).i(i, resolveSlot(slot)).d(d, resolveSlot(slot));
        applyConfig(false);
    }

    @Override
    public void applyHardwareSVAG(int slot, double s, double v, double a, double g) {
        if (motor == null) return;
        this.kS[slot] = s;
        this.kA[slot] = a;
        this.kG[slot] = g;
        motorConfig.closedLoop.apply(new FeedForwardConfig().kV(v, resolveSlot(slot)));
        applyConfig(false);
    }

    @Override
    public void applyHardwareSmartMotion(int slot, double maxVel, double maxAccel, double allowedErr) {
        if (motor == null) return;
        ClosedLoopSlot revSlot = resolveSlot(slot);
        motorConfig.closedLoop.maxMotion.cruiseVelocity(maxVel * 60.0, revSlot)
                                        .maxAcceleration(maxAccel * 60.0, revSlot)
                                        .allowedProfileError(allowedErr, revSlot);
        applyConfig(false);
    }

    @Override
    public void applyHardwareOutputRange(int slot, double min, double max) {
        if (motor == null) return;
        motorConfig.closedLoop.outputRange(min, max, resolveSlot(slot));
        applyConfig(false);
    }

    @Override
    public MotorIOInputs getMotorIOInputs() {
        return inputs;
    }

    // --- Utilidades ---

    private void applyConfig(boolean isInit) {
        ResetMode resetMode = isInit ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters;
        SparkUtil.tryUntilOk(motor, 5, () -> motor.configure(motorConfig, resetMode, PersistMode.kPersistParameters));
    }

    private ClosedLoopSlot resolveSlot(int slotId) {
        return switch (slotId) {
            case 1 -> ClosedLoopSlot.kSlot1;
            case 2 -> ClosedLoopSlot.kSlot2;
            case 3 -> ClosedLoopSlot.kSlot3;
            default -> ClosedLoopSlot.kSlot0;
        };
    }
}