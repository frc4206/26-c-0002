// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSub;
import frc.robot.Robot; 

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToHub_Com extends Command {
  /** Creates a new TurnToHub_Com. */
  CommandSwerveDrivetrain m_drive;
  Robot m_Robot; 
  VisionSub m_vision;

  PhotonTrackedTarget target = new PhotonTrackedTarget(); 
  PhotonCamera camera = new PhotonCamera("frontcam"); 

  PIDController turnPID = new PIDController(5.0, 0.0, 0.05); 

  public TurnToHub_Com(CommandSwerveDrivetrain drive, VisionSub vision, Robot robot) {
    this.m_drive = drive;
    this.m_vision = vision;
    this.m_Robot = robot; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive, m_vision);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if there is no tag visible do not move bot & quit cmd
     if (!m_vision.hasTarget()) {
            m_drive.driveRobotRelative(new ChassisSpeeds());
            return;
        }

        double rotVelo = 0.0; 

        // double yToCent = m_vision.getHubY();
        double yToCent = 0.0d;
        if (Math.abs(yToCent) <= 0.2) {
          rotVelo = 0.0; 
        }
        else {
          rotVelo = turnPID.calculate(yToCent, 0);
        }

        //set the speeds of the robot to the calc'd rotational velo
        m_drive.driveRobotRelative(new ChassisSpeeds(0,0,-rotVelo));
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
