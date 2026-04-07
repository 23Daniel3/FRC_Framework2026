package frc.lib.interfaces.motor;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.logger.LoggedTunableNumber;
import frc.robot.Constants;

public abstract class MotorBase implements MotorIO {
  protected final String name;

  protected MotorControlMode currentMode = MotorControlMode.IDLE;

  protected Angle targetPosition = Rotations.of(0.0);
  protected AngularVelocity targetVelocity = RPM.of(0.0);
  protected final double positionTolerance;
  protected final double velocityTolerance;

  private final boolean tuningMode = Constants.tuningMode;
  private final TunablePID[] pidSlots = new TunablePID[4];
  private final TunableSVAG[] svagSlots = new TunableSVAG[4];
  private final TunableSmart[] smartSlots = new TunableSmart[4];
  private final TunableLimits limits;

  private final MotorController controller =
      new MotorController() {

        @Override
        public void setBrakeMode(boolean enabled) {
          MotorBase.this.setBrakeMode(enabled);
        }

        @Override
        public void setOffset(Angle offset) {
          MotorBase.this.setOffset(offset);
        }

        @Override
        public void runVoltage(Voltage volts) {
          MotorBase.this.runVoltage(volts);
        }

        @Override
        public void runPercentOutput(double percent) {
          MotorBase.this.runPercentOutput(percent);
        }

        @Override
        public void runVelocity(AngularVelocity velocity) {
          MotorBase.this.runVelocity(velocity);
        }

        @Override
        public void runPosition(Angle position) {
          MotorBase.this.runPosition(position);
        }

        @Override
        public void runSmartPosition(Angle position) {
          MotorBase.this.runSmartPosition(position);
        }

        @Override
        public void runVelocity(AngularVelocity velocity, int slot) {
          MotorBase.this.runVelocity(velocity, slot);
        }

        @Override
        public void runPosition(Angle position, int slot) {
          MotorBase.this.runPosition(position, slot);
        }

        @Override
        public void runSmartPosition(Angle position, int slot) {
          MotorBase.this.runSmartPosition(position, slot);
        }

        @Override
        public void stop() {
          MotorBase.this.stop();
        }

        @Override
        public void setCurrentLimit(Current current) {
          MotorBase.this.setCurrentLimit(current);
        }
      };

  public MotorBase(String name, MotorConfig config) {
    this.name = name;

    this.limits = new TunableLimits(name, config.minOutput, config.maxOutput);
    this.positionTolerance = config.positionTolerance.in(Rotations);
    this.velocityTolerance = config.velocityTolerance.in(RPM);

    for (int i = 0; i < 4; i++) {
      pidSlots[i] = new TunablePID(name, i, config.kP[i], config.kI[i], config.kD[i]);
      svagSlots[i] =
          new TunableSVAG(name, i, config.kS[i], config.kV[i], config.kA[i], config.kG[i]);
      smartSlots[i] =
          new TunableSmart(
              name,
              i,
              config.maxMotionMaxVelocity[i].in(RotationsPerSecond),
              config.maxMotionMaxAcceleration[i].in(RotationsPerSecondPerSecond),
              config.maxMotionAllowedClosedLoopError[i].in(Rotations));
    }
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    if (tuningMode) {
      limits.check();
      for (int i = 0; i < 4; i++) {
        pidSlots[i].check();
        svagSlots[i].check();
        smartSlots[i].check();
      }
    }
    updateHardwareInputs(inputs);
    inputs.atSetpoint = calculateAtSetpoint(inputs);
  }

  private class TunablePID {
    private final int id;
    private final LoggedTunableNumber p, i, d;

    TunablePID(String n, int s, double vp, double vi, double vd) {
      this.id = s;
      this.p = new LoggedTunableNumber(n + "/Slot" + s + "/PID/kP", vp);
      this.i = new LoggedTunableNumber(n + "/Slot" + s + "/PID/kI", vi);
      this.d = new LoggedTunableNumber(n + "/Slot" + s + "/PID/kD", vd);
    }

    void check() {
      if (p.hasChanged(hashCode()) || i.hasChanged(hashCode()) || d.hasChanged(hashCode())) {
        applyHardwarePID(id, p.get(), i.get(), d.get());
      }
    }
  }

  private class TunableSVAG {
    private final int id;
    private final LoggedTunableNumber s, v, a, g;

    TunableSVAG(String n, int sl, double vs, double vv, double va, double vg) {
      this.id = sl;
      this.s = new LoggedTunableNumber(n + "/Slot" + sl + "/FF/kS", vs);
      this.v = new LoggedTunableNumber(n + "/Slot" + sl + "/FF/kV", vv);
      this.a = new LoggedTunableNumber(n + "/Slot" + sl + "/FF/kA", va);
      this.g = new LoggedTunableNumber(n + "/Slot" + sl + "/FF/kG", vg);
    }

    void check() {
      if (s.hasChanged(hashCode())
          || v.hasChanged(hashCode())
          || a.hasChanged(hashCode())
          || g.hasChanged(hashCode())) {
        applyHardwareSVAG(id, s.get(), v.get(), a.get(), g.get());
      }
    }
  }

  private class TunableSmart {
    private final int id;
    private final LoggedTunableNumber mv, ma, err;

    TunableSmart(String n, int sl, double v, double a, double e) {
      this.id = sl;
      this.mv = new LoggedTunableNumber(n + "/Slot" + sl + "/Smart/MaxVel", v);
      this.ma = new LoggedTunableNumber(n + "/Slot" + sl + "/Smart/MaxAccel", a);
      this.err = new LoggedTunableNumber(n + "/Slot" + sl + "/Smart/AllowedErr", e);
    }

    void check() {
      if (mv.hasChanged(hashCode()) || ma.hasChanged(hashCode()) || err.hasChanged(hashCode())) {
        applyHardwareSmartMotion(id, mv.get(), ma.get(), err.get());
      }
    }
  }

  private class TunableLimits {
    private final LoggedTunableNumber min, max;

    TunableLimits(String n, double vmin, double vmax) {
      this.min = new LoggedTunableNumber(n + "/Config/MinOutput", vmin);
      this.max = new LoggedTunableNumber(n + "/Config/MaxOutput", vmax);
    }

    void check() {
      if (min.hasChanged(hashCode()) || max.hasChanged(hashCode())) {
        for (int i = 0; i < 4; i++) {
          applyHardwareOutputRange(i, min.get(), max.get());
        }
      }
    }
  }

  protected abstract void updateHardwareInputs(MotorIOInputs inputs);

  @Override
  public MotorController getMotorController() {
    return controller;
  }

  private boolean calculateAtSetpoint(MotorIOInputs inputs) {
    switch (currentMode) {
      case POSITION:
        {
          double error = Math.abs(inputs.position.in(Rotations) - targetPosition.in(Rotations));

          return error < positionTolerance;
        }
      case SMART_POSITION:
        {
          double error = Math.abs(inputs.position.in(Rotations) - targetPosition.in(Rotations));
          double velocity = Math.abs(inputs.velocity.in(RPM));

          return error < positionTolerance && velocity < velocityTolerance;
        }

      case VELOCITY:
        {
          double error = Math.abs(inputs.velocity.in(RPM) - targetVelocity.in(RPM));
          return error < velocityTolerance;
        }

      default:
        return false;
    }
  }
}
