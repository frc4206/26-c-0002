// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.ShooterSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetFlywheelSpeed_Com extends Command {
  /** Creates a new IntakePercent_Com. */
  private ShooterSub m_shooter;
  // double m_targetVelocity;

  private DoubleSupplier m_targetVelocity;

  public SetFlywheelSpeed_Com(ShooterSub shooter, DoubleSupplier targetVelocity) {
    m_shooter = shooter;
    m_targetVelocity = targetVelocity;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setFlywheelSpeed(m_targetVelocity.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("settings speed to " + m_targetVelocity.getAsDouble());
    m_shooter.setFlywheelSpeed(m_targetVelocity.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.shooterMotor1.setControl(new DutyCycleOut(0));
    m_shooter.shooterMotor2.setControl(new DutyCycleOut(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
