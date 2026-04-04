// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ToggleIntakePosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.HopperSub;
// import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.IntakePivotSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.VisionSub;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;
    public double distanceToTarget; 
    private final PhotonCamera camera; 
    public PhotonTrackedTarget currentTarget; 

    IntakePivotSub m_IntakePivotSub; 

    private final CommandXboxController m_driverController = new CommandXboxController(0);


    private final RobotContainer m_robotContainer;

    public VisionSub m_vision; 

    static SwerveModule<TalonFX, TalonFX, CANcoder>[] modules;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        modules = m_robotContainer.drivetrain.getModules();
        camera = new PhotonCamera("frontcam");
    }

    @Override
    public void robotInit() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        Logger.addDataReceiver(new WPILOGWriter()); // write logs to /U/logs
        Logger.addDataReceiver(new NT4Publisher()); // live NT view (optional)

        Logger.start();
        RobotController.setBrownoutVoltage(6.4); 
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 

        // if(ToggleIntakePosition.isDeployed) {
        //      m_IntakePivotSub.intakePivotMotor.setControl(new PositionVoltage(0.0).withSlot(0));

        // } 
        
        // distanceToTarget = getHubY();

        // Logger.recordOutput("Drive/FrontLeftStatorCurrent",
        //         modules[0].getDriveMotor().getStatorCurrent().getValueAsDouble());
        // Logger.recordOutput("Steer/FrontLeftStatorCurrent",
        //         modules[0].getSteerMotor().getStatorCurrent().getValueAsDouble());
        // Logger.recordOutput("Drive/FrontRightStatorCurrent",
        //         modules[1].getDriveMotor().getStatorCurrent().getValueAsDouble());
        // Logger.recordOutput("Steer/FrontRightStatorCurrent",
        //         modules[1].getSteerMotor().getStatorCurrent().getValueAsDouble());
        // Logger.recordOutput("Drive/BackLeftStatorCurrent",
        //         modules[2].getDriveMotor().getStatorCurrent().getValueAsDouble());
        // Logger.recordOutput("Steer/BackLeftStatorCurrent",
        //         modules[2].getSteerMotor().getStatorCurrent().getValueAsDouble());
        // Logger.recordOutput("Drive/BackRightStatorCurrent",
        //         modules[3].getDriveMotor().getStatorCurrent().getValueAsDouble());
        // Logger.recordOutput("Steer/BackRightStatorCurrent",
        //         modules[3].getSteerMotor().getStatorCurrent().getValueAsDouble());

        // Logger.recordOutput("Drive/FrontLeftSupplyCurrent",
        //         modules[0].getDriveMotor().getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput("Steer/FrontLeftSupplyCurrent",
        //         modules[0].getSteerMotor().getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput("Drive/FrontRightSupplyCurrent",
        //         modules[1].getDriveMotor().getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput("Steer/FrontRightSupplyCurrent",
        //         modules[1].getSteerMotor().getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput("Drive/BackLeftSupplyCurrent",
        //         modules[2].getDriveMotor().getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput("Steer/BackLeftSupplyCurrent",
        //         modules[2].getSteerMotor().getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput("Drive/BackRightSupplyCurrent",
        //         modules[3].getDriveMotor().getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput("Steer/BackRightSupplyCurrent",
        //         modules[3].getSteerMotor().getSupplyCurrent().getValueAsDouble());

        
        // HopperSub hopper = m_robotContainer.m_hopper;
        // TalonFX hm1 = hopper.hopperMotor1;
        // TalonFX hm2 = hopper.hopperMotor2;

        // Logger.recordOutput("Hopper/Motor1Current", hm1.getStatorCurrent().getValueAsDouble());
        // Logger.recordOutput("Hopper/Motor2Current", hm2.getStatorCurrent().getValueAsDouble());

        // IntakeSub intake = m_robotContainer.m_intake;
        // TalonFX im1 = intake.intakeRollersMotor1;
        // TalonFX im2 = intake.intakeRollersMotor2;
        // CANcoder intakeCANcoder = intake.intakeCANcoder; 

        // Logger.recordOutput("Intake/EncoderPosition", intakeCANcoder.getPosition().getValueAsDouble());

        // ShooterSub shooter = m_robotContainer.m_shooter;  
        // TalonFX m1 = shooter.shooterMotor1; 
        // TalonFX m2 = shooter.shooterMotor2; 

        // Logger.recordOutput("Shooter/Motor1Current", m1.getStatorCurrent().getValueAsDouble());
        // Logger.recordOutput("Shooter/Motor2CurrentLimit", m2.getStatorCurrent().getValueAsDouble()); 
        // Logger.recordOutput("Shooter/RPM1", m1.getVelocity().getValueAsDouble());
        // Logger.recordOutput("Shooter/RPM2", m2.getVelocity().getValueAsDouble()); 

        

        // Logger.recordOutput("Intake/Motor1Current", im1.getStatorCurrent().getValueAsDouble());
        // Logger.recordOutput("Intake/Motor2Current", im2.getStatorCurrent().getValueAsDouble());

        


    }

    @Override
    public void disabledInit() {} 

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {


    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}

    // public double getHubY() {  
    //     // if ((Math.abs(currentTarget.bestCameraToTarget.getY()) <= 0.1)) {
    //     //     return 0.0; 
    //     // }
    //     // else {
    //     //     return currentTarget.bestCameraToTarget.getY(); 
    //     // }
    //     double targetYaw = 0.0; 
    //     var results = camera.getAllUnreadResults();
    //     if (!results.isEmpty()){
    //         var result = results.get(results.size() - 1); 
    //         if (result.hasTargets()) {
    //             for (var target : result.getTargets()) {
    //                 targetYaw = target.getYaw(); 
    //                 // if (target.getFiducialId() == 25 || target.getFiducialId() == 18 ) {
    //                     if (target.getYaw() <= 0.2) {
    //                         return 0.0; 
    //                     }
    //                 // }
    //                 // currentTarget = target;
    //                 // break;
    //             }
    //         }
    //     }

    //     if (currentTarget.bestCameraToTarget.getY() > 0) {
    //         return currentTarget.bestCameraToTarget.getY() - 0.2; 
    //     }
    //     else {
    //         return currentTarget.bestCameraToTarget.getY() + 0.2; 
    //     }



    //     // if (currentTarget != null && currentTarget.bestCameraToTarget != null && (currentTarget.getFiducialId()==18||currentTarget.getFiducialId()==19||currentTarget.getFiducialId()==20||currentTarget.getFiducialId()==22||currentTarget.getFiducialId()==25)) {
    //     //     return currentTarget.bestCameraToTarget.getY();

    //     // }
    // }
}
