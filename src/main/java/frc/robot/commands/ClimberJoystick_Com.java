// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberJoystick_Com extends Command {
  /** Creates a new IntakePercent_Com. */
  ClimberSub m_climberSub; 
  CommandXboxController m_joystick; 

  public ClimberJoystick_Com(ClimberSub climberSub, CommandXboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climberSub = climberSub; 
    m_joystick = joystick; 
    addRequirements(m_climberSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_joystick.getLeftY()) > 0.1) {
      m_climberSub.setPercentage_func(m_joystick.getRightY());
    } else {
      m_climberSub.setPercentage_func(0);
    }
  }

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
