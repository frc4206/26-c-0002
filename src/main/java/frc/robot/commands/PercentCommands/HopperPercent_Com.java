// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PercentCommands;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HopperPercent_Com extends Command {
  /** Creates a new IntakePercent_Com. */
  HopperSub m_hopperSub;
  double m_percent; 

  public HopperPercent_Com(HopperSub hopperSub, double percent) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_hopperSub = hopperSub; 
    m_percent = percent; 
    addRequirements(hopperSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hopperSub.setPercentage_func(-m_percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopperSub.setPercentage_func(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
