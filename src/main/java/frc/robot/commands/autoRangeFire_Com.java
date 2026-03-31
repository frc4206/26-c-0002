// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.VisionSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class autoRangeFire_Com extends Command {
  /** Creates a new autoRangeFire_Com. */
  ShooterSub m_shooter;
  VisionSub m_vision;
  InterpolatingDoubleTreeMap m_map;
  CommandXboxController driver;

  double previous_rpm;
  double velocity_x;
  double velocity_y;

  public autoRangeFire_Com(ShooterSub shooter, VisionSub vision, 
  CommandXboxController m_driverController, InterpolatingDoubleTreeMap hdssm,
  DoubleSupplier vx, DoubleSupplier vy) {
    m_shooter = shooter;
    m_vision = vision;
    m_map = hdssm;
    this.driver = m_driverController;
    this.velocity_x = vx.getAsDouble();
    this.velocity_y = vy.getAsDouble();

    addRequirements(m_shooter, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previous_rpm = 2000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_vision.latestResult.hasTargets()) {
      double distance = m_vision.getDistanceToHub(velocity_x, velocity_y);

      double rpm = m_map.get(distance);

      // Send RPM instead of distance
      m_shooter.setFlywheelSpeedWithRPM(rpm);
      previous_rpm = rpm;
    } else {
      m_shooter.setFlywheelSpeedWithRPM(previous_rpm);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    previous_rpm = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
