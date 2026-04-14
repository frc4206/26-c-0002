// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.team4206.battleaid.common.TunedJoystick;
import org.team4206.battleaid.common.TunedJoystick.ResponseCurve;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.IncrementSpeedTesting_Com;
import frc.robot.commands.IncrementSpeedUp_Com;
// import frc.robot.commands.IntakeJoystick_Com;
import frc.robot.commands.SetFlywheelSpeed_Com;
import frc.robot.commands.ToggleIntakePosition;
import frc.robot.commands.IntakePivotToPosition;
import frc.robot.commands.autoRangeFire_Com;
import frc.robot.commands.PercentCommands.HopperPercent_Com;
import frc.robot.commands.PercentCommands.IntakePercent_Com;
import frc.robot.commands.PercentCommands.ShooterPercent_Com;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSub;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSub;
import frc.robot.subsystems.IntakePivotSub;
import frc.robot.subsystems.IntakeRollerSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.VisionSub;

public class RobotContainer {
    /* Subsystems */
    public final ShooterSub.Config m_shooterConfig = new ShooterSub.Config("Shooter.toml");
    public final ClimberSub.Config m_climberConfig = new ClimberSub.Config("Climber.toml");
    public final HopperSub.Config m_hopperConfig = new HopperSub.Config("Hopper.toml");

    final IntakePivotSub m_intakepivot = new IntakePivotSub();

    private final IntakeRollerSub m_intakeroller = new IntakeRollerSub();

    public Robot m_Robot;

    // private final PhotonCamera camera;
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final PhotonCamera frontcam = new PhotonCamera("frontcam");
    final VisionSub m_vision = new VisionSub(drivetrain, frontcam);

    final ShooterSub m_shooter = new ShooterSub(m_shooterConfig, m_vision);
    final ClimberSub m_climber = new ClimberSub(m_climberConfig);
    final HopperSub m_hopper = new HopperSub(m_hopperConfig);

    
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1); 

    // then use the same controller handle
    TunedJoystick tunedJoystick = new TunedJoystick(driverController.getHID())
            .useResponseCurve(ResponseCurve.SOFT)
            .setDeadzone(0.1d);

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                        // top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                     // second
                                                                                     // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 3% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                     // motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private double m_targetRPM = 2000; // 2000

    private final SendableChooser<Command> autoChooser;

    // public PhotonCamera camera;
    private AprilTagFieldLayout fieldLayout;

    // h.d.s.s.m. = hub distance shooter speed map
    private InterpolatingDoubleTreeMap hdssm = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap hdftm = new InterpolatingDoubleTreeMap();

    public RobotContainer() {
        // camera = new PhotonCamera("");
        initAprilTags();
        initHubDistanceShooterSpeedMap();
        initHubDistanceFlightTimeMap();

        /* Pathplanner Named Commands */
        /* Basic */
        NamedCommands.registerCommand("Hopper", new HopperPercent_Com(m_hopper, -200).withTimeout(1.5));
        NamedCommands.registerCommand("Flywheels", new SetFlywheelSpeed_Com(m_shooter, () -> 1775).withTimeout(2.0));
        NamedCommands.registerCommand("PivotDown", new IntakePivotToPosition(m_intakepivot, -3.0).withTimeout(1.5));
        NamedCommands.registerCommand("RunIntakeRollers", new IntakePercent_Com(m_intakeroller, 1.0).withTimeout(2.0)); 
        NamedCommands.registerCommand("PivotUp", new IntakePivotToPosition(m_intakepivot, 0.0).withTimeout(2.5));
        NamedCommands.registerCommand("FlywheelsTrench", new SetFlywheelSpeed_Com(m_shooter, () -> 2100).withTimeout(2.0));
        NamedCommands.registerCommand("FlywheelsTrenchLong", new SetFlywheelSpeed_Com(m_shooter, () -> 2100).withTimeout(5.0)); 
        NamedCommands.registerCommand("FlywheelsTrenchMiddle", new SetFlywheelSpeed_Com(m_shooter, () -> 2100).withTimeout(3.0));
        NamedCommands.registerCommand("HopperLong", new HopperPercent_Com(m_hopper, -200).withTimeout(4.5));
        NamedCommands.registerCommand("HopperMiddle", new HopperPercent_Com(m_hopper, -200).withTimeout(3.0)); 
        NamedCommands.registerCommand("FlywheelsTrenchForever", new SetFlywheelSpeed_Com(m_shooter, () -> 2200).withTimeout(20.5));

        /* Trench */
        NamedCommands.registerCommand("RollersShort", new IntakePercent_Com(m_intakeroller, 0.35).withTimeout(3.0)); 

        // camera = new PhotonCamera("frontcam");

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void initHubDistanceShooterSpeedMap() {
        // gotta rebuild the table
        hdssm.put(1.4, 1590.0); // TIME TO FLIGHT: 0.7825 //1440.0 //1540.0 
        hdssm.put(2.1, 1790.0); // TIME TO FLIGHT: 0.9250 //1640.0 //1740.0
        hdssm.put(2.9, 1950.0); // TIME TO FLIGHT: 0.9625 //1800.0 //1900.0
        hdssm.put(3.5, 2150.0); // TIME TO FLIGHT: 1.0750 //2000.0 //2100.0
        hdssm.put(4.0, 2350.0); // TIME TO FLIGHT: 1.1750 //2200.0 //2300.0
        hdssm.put(5.0, 2750.0); // TIME TO FLIGHT: 1.2375 //2600.0 //2700.0

        m_vision.hdssm = this.hdssm;
    }

    public void initHubDistanceFlightTimeMap() {
        hdftm.put(1.4, 0.7825);
        hdftm.put(2.1, 0.9250);
        hdftm.put(2.9, 0.9625);
        hdftm.put(3.5, 1.0750);
        hdftm.put(4.0, 1.1750);
        hdftm.put(5.0, 1.2375);

        m_vision.hdftm = this.hdftm;
    }

    public void initAprilTags() {
        try {
            fieldLayout = new AprilTagFieldLayout(
                    Filesystem.getDeployDirectory().toPath()
                            .resolve("2026-rebuilt-andymark.json"));

            var alliance = DriverStation.getAlliance();
            fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            // if (alliance.isPresent()) {
            // // if (alliance.get() == Alliance.Red) {
            // // fieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
            // // } else {
            // fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            // // }
            // }
            m_vision.setAprilTagField(fieldLayout);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void configureBindings() {

        double auto_aim_speed_modifier = 0.1d;

        /* Driver */
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-tunedJoystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-tunedJoystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-tunedJoystick.getRightX() * MaxAngularRate)));

        driverController.leftTrigger().whileTrue(
                drivetrain.applyRequest(() -> {
                    double vx = -tunedJoystick.getLeftY() * MaxSpeed * auto_aim_speed_modifier;
                    double vy = -tunedJoystick.getLeftX() * MaxSpeed * auto_aim_speed_modifier;

                    double omega = m_vision.getRotationToHub(drivetrain, vx, vy, hdftm)
                            * MaxAngularRate;
                    // System.out.println("Distnace to HUB: " + m_vision.getDistanceToHub());

                    return drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega)
                            .withRotationalDeadband(0);
                }));

        // driverController.b().toggleOnTrue(new ToggleIntakePosition(m_intakepivot, false));
        driverController.a().onTrue(new IntakePivotToPosition(m_intakepivot, 0.0)); 
        driverController.b().onTrue(new IntakePivotToPosition(m_intakepivot, -3.1)); 


        driverController.rightTrigger().toggleOnTrue(new IntakePercent_Com(m_intakeroller, 1.0));

        driverController.x().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldCentric();
            drivetrain.getPigeon2().reset();
        }));

        /* Operator */
        operatorController.pov(0).onTrue(new SetFlywheelSpeed_Com(m_shooter, () -> 2750.0)); //up on dpad - to shoot from corner
        operatorController.y().onTrue(new SetFlywheelSpeed_Com(m_shooter, () -> 0.0)); 
        operatorController.a().onTrue(new SetFlywheelSpeed_Com(m_shooter, () -> 2000)); //to shoot from general radius 
        operatorController.rightTrigger().toggleOnTrue(new HopperPercent_Com(m_hopper, -200)); 

        operatorController.leftTrigger().whileTrue(
                new ParallelCommandGroup(
                        // Shooting and auto-aiming runs uninterrupted
                        new autoRangeFire_Com(
                                m_shooter,
                                m_vision,
                                driverController,
                                hdssm,
                                () -> -tunedJoystick.getLeftY() * MaxSpeed * auto_aim_speed_modifier, // vx lambda
                                () -> -tunedJoystick.getLeftX() * MaxSpeed * auto_aim_speed_modifier // vy lambda
                        ),
                        // Hopper spins after a small delay
                        new SequentialCommandGroup(
                                new WaitCommand(0.50),
                                new HopperPercent_Com(m_hopper, -200))
                        //intake is out of the sequence bc it's slow and we like controlling it pls don't freak out this was on purpose
                        // Toggle intake runs independently after 1.2 seconds
                        // new SequentialCommandGroup(
                        //         new WaitCommand(0.8),
                        //         new InstantCommand(() -> edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance()
                        //                 .schedule(new ToggleIntakePosition(m_intakepivot, true)))
                        ));

        
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        // // Reset our field centric heading to match the robot
        // // facing away from our alliance station wall (0 deg).
        // drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        // // Then slowly drive forward (away from us) for 5 seconds.
        // drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
        // .withVelocityY(0)
        // .withRotationalRate(0))
        // .withTimeout(5.0),
        // // Finally idle for the rest of auton
        // drivetrain.applyRequest(() -> idle));

        return autoChooser.getSelected();
    }
}
