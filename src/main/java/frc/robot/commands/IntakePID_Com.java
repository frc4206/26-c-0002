// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import com.ctre.phoenix6.controls.DutyCycleOut;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.IntakeSub;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class IntakePID_Com extends Command {
//   /** Creates a new IntakePercent_Com. */
//   IntakeSub m_intakeSub; 
//   double m_intakePosition; 

//   public IntakePID_Com(IntakeSub intakeSub, double intakePosition) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     m_intakeSub = intakeSub; 
//     m_intakePosition = intakePosition; 
//     addRequirements(intakeSub);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_intakeSub.setPosition_func(m_intakePosition);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_intakeSub.intakePivotMotor.setControl(new DutyCycleOut(0)); 
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
