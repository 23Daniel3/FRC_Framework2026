package frc.lib.interfaces.motor;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MotorIOSim extends MotorBase {
  // Simulação física
  private final DCMotorSim sim;
  private final MotorIOInputs inputs = new MotorIOInputs();

  // Controladores para simular o comportamento interno do Spark
  private final PIDController[] pids = new PIDController[4];
  private final SimpleMotorFeedforward[] ffs = new SimpleMotorFeedforward[4];

  private double appliedVolts = 0.0;

  /**
   * Construtor para simulação do motor. * @param name Nome para log
   *
   * @param config Configurações iniciais
   * @param motor Modelo do motor (ex: DCMotor.getNeoVortex(1))
   * @param gearing Redução (ex: 10.0 para 10:1)
   * @param jInertia Momento de inércia em Kg*m^2 (ex: 0.005 para um Intake leve)
   */
  public MotorIOSim(
      String name, MotorConfig config, DCMotor motor, double gearing, double jInertia) {
    super(name, config);

    // CORREÇÃO AQUI: Cria o sistema linear primeiro usando a API moderna da WPILib
    var plant = LinearSystemId.createDCMotorSystem(motor, jInertia, gearing);
    this.sim = new DCMotorSim(plant, motor);

    // Inicializa PIDs e FFs para os 4 slots
    for (int i = 0; i < 4; i++) {
      pids[i] = new PIDController(config.kP[i], config.kI[i], config.kD[i]);
      // Na simulação o feedforward simples usa apenas kV
      ffs[i] = new SimpleMotorFeedforward(0, config.kV[i]);
    }
  }

  @Override
  protected void updateHardwareInputs(MotorIOInputs inputs) {
    // Atualiza a simulação (considerando ciclo padrão de 20ms)
    sim.update(0.020);

    // Preenche os inputs com dados da simulação
    inputs.position = Rotations.of(sim.getAngularPositionRotations());
    inputs.velocity = RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
    inputs.appliedVolts = Volts.of(appliedVolts);
    inputs.current = Amps.of(sim.getCurrentDrawAmps());
    inputs.temperature = Celsius.of(40.0); // Simulação estática de temperatura
    inputs.isConnected = true;
    inputs.activeFaults = new String[] {};

    // Lógica simples de "atSetpoint" para a FSM não travar
    if (currentMode == MotorControlMode.POSITION
        || currentMode == MotorControlMode.SMART_POSITION) {
      if (targetPosition != null) {
        inputs.atSetpoint = Math.abs(inputs.position.minus(targetPosition).in(Rotations)) < 0.05;
      }
    } else if (currentMode == MotorControlMode.VELOCITY) {
      if (targetVelocity != null) {
        inputs.atSetpoint = Math.abs(inputs.velocity.minus(targetVelocity).in(RPM)) < 10;
      }
    }
  }

  @Override
  public void runVoltage(Voltage volts) {
    currentMode = MotorControlMode.VOLTAGE;
    // Limita a voltagem para +/- 12V
    appliedVolts = Math.max(-12.0, Math.min(12.0, volts.in(Volts)));
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void runPercentOutput(double percent) {
    runVoltage(Volts.of(percent * 12.0));
  }

  @Override
  public void runVelocity(AngularVelocity velocity, int slot) {
    currentMode = MotorControlMode.VELOCITY;
    targetVelocity = velocity;

    double ffEffort = ffs[slot].calculate(velocity.in(RadiansPerSecond));
    double pidEffort =
        pids[slot].calculate(sim.getAngularVelocityRadPerSec(), velocity.in(RadiansPerSecond));

    // Atualiza a voltagem considerando limites e aplica à simulação
    runVoltage(Volts.of(ffEffort + pidEffort));
  }

  @Override
  public void runPosition(Angle position, int slot) {
    currentMode = MotorControlMode.POSITION;
    targetPosition = position;

    double pidEffort =
        pids[slot].calculate(sim.getAngularPositionRotations(), position.in(Rotations));
    runVoltage(Volts.of(pidEffort));
  }

  @Override
  public void runSmartPosition(Angle position, int slot) {
    // Para uma simulação simples e rápida, tratamos SmartPosition como Position comum
    // Para melhorar isso no futuro, você precisaria implementar um TrapezoidProfile
    runPosition(position, slot);
    currentMode = MotorControlMode.SMART_POSITION;
  }

  @Override
  public void stop() {
    currentMode = MotorControlMode.IDLE;
    runVoltage(Volts.of(0));
  }

  @Override
  public void applyHardwarePID(int slot, double p, double i, double d) {
    pids[slot].setPID(p, i, d);
  }

  @Override
  public void applyHardwareSVAG(int slot, double s, double v, double a, double g) {
    ffs[slot] = new SimpleMotorFeedforward(s, v, a);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    // DCMotorSim não simula perfeitamente coast mode (ele sempre assume um decaimento resistivo).
    // Para fins de teste rápido, deixamos vazio.
  }

  @Override
  public void setOffset(Angle offset) {
    sim.setState(offset.in(Radians), sim.getAngularVelocityRadPerSec());
  }

  @Override
  public MotorIOInputs getMotorIOInputs() {
    return inputs;
  }

  // Implementações obrigatórias de métodos sem slot da interface (delegam para o slot 0)
  @Override
  public void runVelocity(AngularVelocity v) {
    runVelocity(v, 0);
  }

  @Override
  public void runPosition(Angle p) {
    runPosition(p, 0);
  }

  @Override
  public void runSmartPosition(Angle p) {
    runSmartPosition(p, 0);
  }

  @Override
  public void applyHardwareSmartMotion(
      int slot, double maxVel, double maxAccel, double allowedErr) {
    // Na simulação simples, ignoramos o profile de velocidade (apenas para testes rápidos)
  }

  @Override
  public void applyHardwareOutputRange(int slot, double min, double max) {
    // Limitador simples (não estritamente necessário para testes de lógica básica)
  }

  @Override
  public void setCurrentLimit(Current current) {
    // Não simula limite de corrente
  }

  @Override
  public MotorController getMotorController() {
    return new MotorController() {
      @Override
      public void setBrakeMode(boolean enabled) {
        MotorIOSim.this.setBrakeMode(enabled);
      }

      @Override
      public void setOffset(Angle offset) {
        MotorIOSim.this.setOffset(offset);
      }

      @Override
      public void runVoltage(Voltage volts) {
        MotorIOSim.this.runVoltage(volts);
      }

      @Override
      public void runPercentOutput(double percent) {
        MotorIOSim.this.runPercentOutput(percent);
      }

      @Override
      public void runVelocity(AngularVelocity velocity) {
        MotorIOSim.this.runVelocity(velocity);
      }

      @Override
      public void runPosition(Angle position) {
        MotorIOSim.this.runPosition(position);
      }

      @Override
      public void runSmartPosition(Angle position) {
        MotorIOSim.this.runSmartPosition(position);
      }

      @Override
      public void runVelocity(AngularVelocity velocity, int slot) {
        MotorIOSim.this.runVelocity(velocity, slot);
      }

      @Override
      public void runPosition(Angle position, int slot) {
        MotorIOSim.this.runPosition(position, slot);
      }

      @Override
      public void runSmartPosition(Angle position, int slot) {
        MotorIOSim.this.runSmartPosition(position, slot);
      }

      @Override
      public void stop() {
        MotorIOSim.this.stop();
      }

      @Override
      public void setCurrentLimit(Current current) {
        MotorIOSim.this.setCurrentLimit(current);
      }
    };
  }
}
