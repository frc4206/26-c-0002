package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ConfigTalonFX;

public class IntakeRollerSub extends SubsystemBase {

    public static class Config extends LoadableConfig {
        public double intakePercent;
        public double outtakePercent;

        public Config(String filename) {
            super.load(this, filename);
        }
    }

    ConfigTalonFX.Config intakeRollersMotor1Config = new ConfigTalonFX.Config("IntakeRollersMotor1.toml");
    ConfigTalonFX.Config intakeRollersMotor2Config = new ConfigTalonFX.Config("IntakeRollersMotor2.toml");

    public TalonFX intakeRollersMotor1 = new TalonFX(intakeRollersMotor1Config.canID, "rio");
    public TalonFX intakeRollersMotor2 = new TalonFX(intakeRollersMotor2Config.canID, "rio");

    TalonFXConfiguration motor1Config = new TalonFXConfiguration();
    TalonFXConfiguration motor2Config = new TalonFXConfiguration();

    public IntakeRollerSub() {
        motor1Config.CurrentLimits.SupplyCurrentLimit = 30;
        motor1Config.CurrentLimits.SupplyCurrentLimitEnable = true;
        motor1Config.CurrentLimits.StatorCurrentLimit = 40;
        motor1Config.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeRollersMotor1.getConfigurator().apply(motor1Config);

        motor2Config.CurrentLimits.SupplyCurrentLimit = 30;
        motor2Config.CurrentLimits.SupplyCurrentLimitEnable = true;
        motor2Config.CurrentLimits.StatorCurrentLimit = 40;
        motor2Config.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeRollersMotor2.getConfigurator().apply(motor2Config);
    }

    public void setPercentageRollers(double percentage) {
        intakeRollersMotor1.setControl(new DutyCycleOut(percentage));
        intakeRollersMotor2.setControl(new DutyCycleOut(-percentage));
    }

    public void setPercentageRollers_func(double percentage) {
        intakeRollersMotor1.setControl(new DutyCycleOut(percentage));
        intakeRollersMotor2.setControl(new DutyCycleOut(-percentage));
    }

    @Override
    public void periodic() {
    }
}