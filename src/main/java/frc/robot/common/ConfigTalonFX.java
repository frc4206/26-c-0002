package frc.robot.common;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class ConfigTalonFX {

    Config cfg;

    // Creates Motor
    public TalonFX motor;

    public TalonFXConfiguration talonConfigs;

    public ConfigTalonFX(ConfigTalonFX.Config cfg, TalonFX motor) {
        this.cfg = cfg;

        this.motor = motor;
        talonConfigs = new TalonFXConfiguration();
        // motor.setInverted(cfg.inverted);     //Commented out bc it was breaking the code and I'm pretty sure it's okay if we don't have it 

        
        if (cfg.isBreakMode) {
            motor.setNeutralMode(NeutralModeValue.Brake);
        } else {
            motor.setNeutralMode(NeutralModeValue.Coast);
        }

        applyConfigs();
    }

    public static class Slot extends LoadableConfig {
        public double kp; // proportional
        public double ki; // integral
        public double kd; // derivative

        public double ks;
        public double kv;
        public double ka;

        public Slot() {
        };
    }

    public static final class Config extends LoadableConfig {
        public String name;
        @Required
        public int canID;
        @Required
        public boolean inverted;
        @Required
        public boolean isBreakMode;

        public Slot slot0;
        public Slot slot1;
        public Slot slot2;

        public double kCruiseVelocity;
        public double kAcceleration;
        public double kJerk;
        public double kSupplyCurrentLimit;
        public double kStatorCurrentLimit;

        public double intakelimit;
        public double shootlimit;
        public double climblimit;
        public double defenselimit;
        public double cyclelimit;

        public Config(String filename) {
            super.load(this, filename);
            // LoadableConfig.print(this);
        }
    }

    public void setSlot0(Slot bs) {
        talonConfigs.Slot0.kP = bs.kp;
        talonConfigs.Slot0.kI = bs.ki;
        talonConfigs.Slot0.kD = bs.kd;
    }

    public void setSlot0SVA(Slot s) {
        talonConfigs.Slot0.kS = s.ks;
        talonConfigs.Slot0.kV = s.kv;
        talonConfigs.Slot0.kA = s.ka;
    }

    public void setSlot1(Slot bs) {
        talonConfigs.Slot1.kP = bs.kp;
        talonConfigs.Slot1.kI = bs.ki;
        talonConfigs.Slot1.kD = bs.kd;
    }

    public void setSlot2(Slot bs) {
        talonConfigs.Slot2.kP = bs.kp;
        talonConfigs.Slot2.kI = bs.ki;
        talonConfigs.Slot2.kD = bs.kd;
    }

    public void applyTrapezoidalMotionProfile() {
        // talonConfigs.MotionMagic.MotionMagicCruiseVelocity = cfg.kCruiseVelocity;
        // talonConfigs.MotionMagic.MotionMagicAcceleration = cfg.kAcceleration;
        // talonConfigs.MotionMagic.MotionMagicJerk = cfg.kJerk;
        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = cfg.kCruiseVelocity; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = cfg.kAcceleration; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = cfg.kJerk;
        // talonConfigs.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(cfg.kSupplyCurrentLimit));
        // talonConfigs.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(cfg.kStatorCurrentLimit));

        motor.getConfigurator().apply(talonConfigs);
    }

    public void applyConfigs() {
        motor.getConfigurator().apply(talonConfigs);
    }
}