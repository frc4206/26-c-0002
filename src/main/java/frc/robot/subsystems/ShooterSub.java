// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.compound.Diff_VelocityDutyCycle_Velocity;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ConfigTalonFX;
import frc.robot.common.ConfigTalonFX.Config;

import edu.wpi.first.wpilibj.Timer;

public class ShooterSub extends SubsystemBase {
  /** Creates a new ShooterSub. */
  public double targetSpeed = 0.5;

  public double VOLTAGE_TO_OVERCOME_STATIC_FRICTION = 0.26d; // VOLTS, tested by hand, DO NOT change
  public double VOLTAGE_TO_MAINTAIN_SPEED = 0.13d;

  /* Configs */
  ConfigTalonFX.Config shooterMotor1Config = new ConfigTalonFX.Config("ShooterMotor1.toml");
  ConfigTalonFX.Config shooterMotor2Config = new ConfigTalonFX.Config("ShooterMotor2.toml");

  public Config shooterConfig;

  /* Motors */
  public TalonFX shooterMotor1 = new TalonFX(shooterMotor1Config.canID, "rio");
  public TalonFX shooterMotor2 = new TalonFX(shooterMotor2Config.canID, "rio");

  // ConfigTalonFX shooterMotor1Apply = new ConfigTalonFX(shooterMotor1Config,
  // shooterMotor1);
  // ConfigTalonFX shooterMotor2Apply = new ConfigTalonFX(shooterMotor2Config,
  // shooterMotor2);

  TalonFXConfiguration motor1config = new TalonFXConfiguration();
  TalonFXConfiguration motor2config = new TalonFXConfiguration();

  public InterpolatingDoubleTreeMap autoRangeMap = new InterpolatingDoubleTreeMap();

  private double m_lastPrintTime = 0;

  public static class Config extends LoadableConfig {

    /* Misc */
    public double outtakePercent; // placeholder until treetable stuff happens

    public Config(String filename) {
      super.load(this, filename);
      // LoadableConfig.print(this);
    }
  }

  VisionSub m_vision;

  public ShooterSub(Config shooterConfig, VisionSub vision) {
    this.shooterConfig = shooterConfig;

    this.m_vision = vision;

    // shooterMotor1Apply.applyConfigs();

    motor1config.Slot0.kS = VOLTAGE_TO_OVERCOME_STATIC_FRICTION;
    motor1config.Slot0.kV = VOLTAGE_TO_MAINTAIN_SPEED;
    motor1config.Slot0.kP = 0.72d;
    motor1config.Slot0.kI = 0.0d;
    motor1config.Slot0.kD = 0.0d;
    motor1config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor1config.CurrentLimits.SupplyCurrentLimit = 80;
    motor1config.CurrentLimits.SupplyCurrentLimitEnable = true;
    motor1config.CurrentLimits.StatorCurrentLimit = 130;
    motor1config.CurrentLimits.StatorCurrentLimitEnable = true;

    motor1config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterMotor1.getConfigurator().apply(motor1config);

    motor2config.Slot0.kS = VOLTAGE_TO_OVERCOME_STATIC_FRICTION;
    motor2config.Slot0.kV = VOLTAGE_TO_MAINTAIN_SPEED;
    motor2config.Slot0.kP = 0.72d;
    motor2config.Slot0.kI = 0.0d;
    motor2config.Slot0.kD = 0.0d;
    motor2config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor2config.CurrentLimits.SupplyCurrentLimit = 80;
    motor2config.CurrentLimits.SupplyCurrentLimitEnable = true;
    motor2config.CurrentLimits.StatorCurrentLimit = 130;
    motor2config.CurrentLimits.StatorCurrentLimitEnable = true;

    motor2config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterMotor2.getConfigurator().apply(motor2config);

    // TODO: put this interpolation table into a seperate file!
    autoRangeMap.put(1.41, 1300.0);
    autoRangeMap.put(1.53, 1400.0);
    autoRangeMap.put(1.83, 1450.0);
    autoRangeMap.put(2.12, 1500.0);
    autoRangeMap.put(2.6, 1900.0);
    autoRangeMap.put(2.89, 2000.0);
    autoRangeMap.put(3.06,2500.0);
    autoRangeMap.put(3.38,2075.0);
    autoRangeMap.put(3.82,2300.0);
    autoRangeMap.put(4.45,2525.0);
    // dutyRangeMap.put(2.50, 0.36);
    // dutyRangeMap.put(2.41, 0.35);
    // dutyRangeMap.put(2.31, 0.34);
    // dutyRangeMap.put(2.22, 0.33);
    // dutyRangeMap.put(2.12, 0.33);
    // dutyRangeMap.put(2.01, 0.33);
    // dutyRangeMap.put(1.90, 0.33);
    // dutyRangeMap.put(1.78, 0.33);
    // dutyRangeMap.put(1.67, 0.31);
    // dutyRangeMap.put(1.56, 0.31);
    // dutyRangeMap.put(1.43, 0.31);
    // dutyRangeMap.put(1.32, 0.31);
    // dutyRangeMap.put(1.21, 0.31);
    // dutyRangeMap.put(2.90, 0.36);
    // dutyRangeMap.put(3.00, 0.39);
    // dutyRangeMap.put(3.15, 0.40);
    // dutyRangeMap.put(3.20, 0.40);
    // dutyRangeMap.put(3.70, 0.42);
    // dutyRangeMap.put(2.70, 0.35);
    // dutyRangeMap.put(2.60, 0.34);
    // dutyRangeMap.put(2.50, 0.34);
    // dutyRangeMap.put(3.30, 0.41);
    // dutyRangeMap.put(3.40, 0.41);
    // dutyRangeMap.put(4.50, 0.51);

  }

  public void setPercentage_func(double percentage) {
    shooterMotor1.set(percentage);
    shooterMotor2.set(percentage);
  }

  public void incrementSpeedTesting() {
    // shooterMotor1.set(targetSpeed);
    // shooterMotor2.set(-targetSpeed);
    shooterMotor1.setControl(new DutyCycleOut(targetSpeed));
    shooterMotor2.setControl(new DutyCycleOut(targetSpeed));
  }

  public void autoRangeFire_dist_func(double distance) {

    shooterMotor1.setControl(new DutyCycleOut(autoRangeMap.get(distance)));
    shooterMotor2.setControl(new DutyCycleOut(autoRangeMap.get(distance)));
    // for (int i = 0; i < 10; i++) {
    //   System.out.println("planned speed: " + autoRangeMap.get(distance));
    // }
  }

  public void autoRangeFire_func(double distance) {
    shooterMotor1.setControl(new VelocityVoltage(autoRangeMap.get(distance) / 60).withSlot(0));
    shooterMotor2.setControl(new VelocityVoltage(autoRangeMap.get(distance) / 60).withSlot(0));
  }

  public void setFlywheelSpeedWithRPM(double rpm)
  {
    shooterMotor1.setControl(new VelocityVoltage(rpm / 60).withSlot(0));
    shooterMotor2.setControl(new VelocityVoltage(rpm / 60).withSlot(0)); 
  }

  public void incrementSpeedUp(double increment) {
    targetSpeed += increment;
    // System.out.println("VELOCITY: " + targetSpeed);
  }

  public void setFlywheelSpeed(double velocity) {
    // the first VelocityVoltage value is 0 because the withVelocity value replaces
    // it anyway,
    // and the withVelocity value is necessary for feedforward. If you take
    // feedforward out
    // then just delete until withSlot and replace the '0' with 'velocity'
    velocity = velocity / 60; // to get to rps from rpm

    double currentTime = Timer.getFPGATimestamp();

    if (m_vision.hasTarget() && currentTime - m_lastPrintTime >= 0.5) {
      double distance = m_vision.getTargetX();
      System.out.println("Distance: " + distance);
      m_lastPrintTime = currentTime;
    }

    shooterMotor1.setControl(new VelocityVoltage(velocity).withSlot(0));
    shooterMotor2.setControl(new VelocityVoltage(velocity).withSlot(0));
  }

  public void setFlywheelVoltage(double voltage) {
    VoltageOut voltageRequest = new VoltageOut(0.1);
    shooterMotor1.setControl(voltageRequest.withOutput(voltage));
    shooterMotor2.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("VELOCITY: " + targetSpeed);
    // System.out.println(m_vision.getTargetX() + ", " + shooterMotor1.getVelocity().getValueAsDouble()*60); 
    // System.out.println(m_vision.getYaw()); 
  }

  /** Set the target velocity of the shooter in motor units per 100ms */
  public void setFlywheelVelocity(double targetRPM) {
    // Convert RPM to TalonFX native units (ticks per 100ms)
    double ticksPer100ms = rpmToTicksPer100ms(targetRPM);

    // Use TalonFX VelocityVoltage control
    VelocityVoltage control = new VelocityVoltage(ticksPer100ms);
    shooterMotor1.setControl(control);
    shooterMotor2.setControl(control);
  }

  /** Stop the shooter motors */
  public void stop() {
    shooterMotor1.setControl(new VoltageOut(0));
    shooterMotor2.setControl(new VoltageOut(0));
  }

  /** Helper: Convert RPM to TalonFX units per 100ms */
  private double rpmToTicksPer100ms(double rpm) {
    double ticksPerRev = 2048.0; // Falcon 500 encoder ticks per revolution
    return rpm * ticksPerRev / 600.0; // 600 = 60s * 10 (100ms per tick)
  }

  /** Optional: get current RPM for debugging / PID monitoring */
  // public double getCurrentRPM() {
  // double sensorVel1 = shooterMotor1.getRotorVelocity().getValue();
  // return sensorVelToRPM(sensorVel1);
  // }

  // private double sensorVelToRPM(double ticksPer100ms) {
  // return ticksPer100ms * 600.0 / 2048.0;
  // }
}
