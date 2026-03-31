// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ConfigTalonFX;
import frc.robot.common.ConfigTalonFX.Config;

public class ClimberSub extends SubsystemBase {
  /** Creates a new ClimberSub. */
  /* Configs */
  ConfigTalonFX.Config climberMotor1Config = new ConfigTalonFX.Config("ClimberMotor1.toml");
  ConfigTalonFX.Config climberMotor2Config = new ConfigTalonFX.Config("ClimberMotor2.toml"); 

  public Config climberConfig; 

  /* Motors */
  public TalonFX climberMotor1 = new TalonFX(climberMotor1Config.canID, "rio"); 
  public TalonFX climberMotor2 = new TalonFX(climberMotor2Config.canID, "rio"); 

  ConfigTalonFX climberMotor1CFGapply = new ConfigTalonFX(climberMotor1Config, climberMotor1);
  ConfigTalonFX climberMotor2CFGapply = new ConfigTalonFX(climberMotor2Config, climberMotor1);

  public static class Config extends LoadableConfig {
    
    /* Positions */
    public double stowPosition; 
    public double extendedPosition; 
    public double climbPosition; 
    
    public Config(String filename) {
      super.load(this, filename); 
      //LoadableConfig.print(this); 
    }
  }

  public ClimberSub(Config climberConfig) {
    this.climberConfig = climberConfig; 

    climberMotor1CFGapply.setSlot0(climberMotor1Config.slot0);
    climberMotor2CFGapply.setSlot0(climberMotor2Config.slot0);
    climberMotor1CFGapply.applyConfigs();
    climberMotor2CFGapply.applyConfigs();
  }

  public void setPercentage_func(double percentage) {
    climberMotor1.setControl(new DutyCycleOut(percentage)); 
    climberMotor2.setControl(new DutyCycleOut(-percentage));
  }

  public void setPosition_func(double pos) {
    //climberMotor2.setPosition(climberMotor1.getPosition().getValueAsDouble()); 
    climberMotor1.setControl(new PositionVoltage(0).withPosition(pos).withSlot(0)); 
    climberMotor2.setControl(new PositionVoltage(0).withPosition(pos).withSlot(0)); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
