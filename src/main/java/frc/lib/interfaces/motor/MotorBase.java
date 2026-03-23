package frc.lib.interfaces.motor;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.lib.logger.LoggedTunableNumber;

public abstract class MotorBase implements MotorIO {
    protected final String name;
    protected final boolean tuningMode;

    private final TunablePID[] pidSlots = new TunablePID[4];
    private final TunableSVAG[] svagSlots = new TunableSVAG[4];
    private final TunableSmart[] smartSlots = new TunableSmart[4];
    private final TunableLimits limits;

    public MotorBase(String name, boolean tuningMode, MotorConfig config) {
        this.name = name;
        this.tuningMode = tuningMode;

        this.limits = new TunableLimits(name, config.minOutput, config.maxOutput);

        for (int i = 0; i < 4; i++) {
            pidSlots[i] = new TunablePID(name, i, config.kP[i], config.kI[i], config.kD[i]);
            svagSlots[i] = new TunableSVAG(name, i, config.kS[i], config.kV[i], config.kA[i], config.kG[i]);
            smartSlots[i] = new TunableSmart(name, i, 
                config.maxMotionMaxVelocity[i].in(RotationsPerSecond),
                config.maxMotionMaxAcceleration[i].in(RotationsPerSecondPerSecond),
                config.maxMotionAllowedClosedLoopError[i].in(Rotations)
            );
        }
        
        setBrakeMode(config.idleMode == IdleMode.kBrake);
        setCurrentLimit(config.currentLimit);
        
        for (int i = 0; i < 4; i++) {
            applyHardwareOutputRange(i, config.minOutput, config.maxOutput);
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
            if (s.hasChanged(hashCode()) || v.hasChanged(hashCode()) || a.hasChanged(hashCode()) || g.hasChanged(hashCode())) {
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
}