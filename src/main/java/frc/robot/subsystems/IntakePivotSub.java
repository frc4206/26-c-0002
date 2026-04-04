package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ToggleIntakePosition;
import frc.robot.common.ConfigTalonFX;

public class IntakePivotSub extends SubsystemBase {

    public static class Config extends LoadableConfig {
        public double stowPosition;
        public double deployPosition;

        public Config(String filename) {
            super.load(this, filename);
        }
    }

    ConfigTalonFX.Config intakePivotMotorConfig = new ConfigTalonFX.Config("IntakePivotMotor.toml");
    public TalonFX intakePivotMotor = new TalonFX(intakePivotMotorConfig.canID, "rio");
    TalonFXConfiguration intakePivotConfig = new TalonFXConfiguration();

    public IntakePivotSub() {
        intakePivotConfig.CurrentLimits.SupplyCurrentLimit = 30;
        intakePivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakePivotConfig.CurrentLimits.StatorCurrentLimit = 120;
        intakePivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakePivotMotor.getConfigurator().apply(intakePivotConfig);
        

        intakePivotConfig.Slot0.kG = 1.05d;
        intakePivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        intakePivotConfig.Feedback.RotorToSensorRatio = 9.63;
        intakePivotConfig.Slot0.kP = 5.0; //2.0 in last match
        intakePivotConfig.Slot0.kI = 0;
        intakePivotConfig.Slot0.kD = 0.2;
        intakePivotConfig.Slot0.kS = -0.05;

        intakePivotConfig.Slot1.kG = 1.05d; 
        intakePivotConfig.Slot1.GravityType = GravityTypeValue.Arm_Cosine; 
        intakePivotConfig.Slot1.kP = 2.0; 
        intakePivotConfig.Slot1.kI = 0.0; 
        intakePivotConfig.Slot1.kD = 0.2; 
        intakePivotConfig.Slot1.kS = -0.05; 

        intakePivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakePivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakePivotMotor.getConfigurator().apply(intakePivotConfig);
        intakePivotMotor.setPosition(0.0);
    }

    //yeah i know i switched them but it works 
    //USE FOR DEPLOYING 
    public void setPositionOfIntakePivot(double rotations) {
        intakePivotMotor.setControl(new PositionVoltage(0).withPosition(rotations).withSlot(0));
    }

    //USE FOR STOWING 
    public void setPosOfIntakeSlot1(double rotations) {
        intakePivotMotor.setControl(new PositionVoltage(0).withPosition(rotations).withSlot(1)); 
    }

    public void setPercentagePivot(double percentage) {
        intakePivotMotor.setControl(new DutyCycleOut(percentage));
    }

    public double getPosition() {
        return intakePivotMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {}
}