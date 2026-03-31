// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.IntakeSub;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class IntakeJoystick_Com extends Command {
//   /** Creates a new IntakePercent_Com. */
//   IntakeSub m_intakeSub; 
//   CommandXboxController m_joystick; 

//   public IntakeJoystick_Com(IntakeSub intakeSub, CommandXboxController joystick) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     m_intakeSub = intakeSub; 
//     m_joystick = joystick; 
//     addRequirements(m_intakeSub);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (Math.abs(m_joystick.getRightY()) > 0.1) {
//       m_intakeSub.setPercentagePivot_func(m_joystick.getRightY());
//     } else {
//       m_intakeSub.setPercentagePivot_func(0);
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_intakeSub.setPercentagePivot_func(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
