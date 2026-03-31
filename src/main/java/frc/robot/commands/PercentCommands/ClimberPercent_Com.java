// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PercentCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberPercent_Com extends Command {
  /** Creates a new IntakePercent_Com. */
  ClimberSub m_climberSub;  
  double m_percent; 

  public ClimberPercent_Com(ClimberSub climberSub, double percent) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climberSub = climberSub; 
    m_percent = percent; 
    addRequirements(climberSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climberSub.setPercentage_func(m_percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSub.setPercentage_func(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
