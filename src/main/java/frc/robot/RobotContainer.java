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

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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

    private final IntakePivotSub m_intakepivot = new IntakePivotSub();

    private final IntakeRollerSub m_intakeroller = new IntakeRollerSub();

    public Robot m_Robot;

    // private final PhotonCamera camera;
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final PhotonCamera frontcam = new PhotonCamera("frontcam");
    final VisionSub m_vision = new VisionSub(drivetrain, frontcam);

    final ShooterSub m_shooter = new ShooterSub(m_shooterConfig, m_vision);
    final ClimberSub m_climber = new ClimberSub(m_climberConfig);
    final HopperSub m_hopper = new HopperSub(m_hopperConfig);

    /* Joysticks */
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);
    private final CommandXboxController m_testingController = new CommandXboxController(2);
    private final CommandXboxController m_climberController = new CommandXboxController(3);
    private final CommandXboxController m_soloTestController = new CommandXboxController(5);

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 3% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

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
        /* Basic: */
        NamedCommands.registerCommand("Hopper", new HopperPercent_Com(m_hopper, 0.80).withTimeout(2.5));
        NamedCommands.registerCommand("Flywheels", new SetFlywheelSpeed_Com(m_shooter, () -> 1775).withTimeout(4.5));

        /* Human Player Auto: */
        NamedCommands.registerCommand("FlywheelsTrench", new ShooterPercent_Com(m_shooter, .51).withTimeout(5.0));
        NamedCommands.registerCommand("HopperShort", new HopperPercent_Com(m_hopper, 0.90).withTimeout(5));

        // camera = new PhotonCamera("frontcam");

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void initHubDistanceShooterSpeedMap() {
        // gotta rebuild the table
        hdssm.put(1.4, 1675.0); // TIME TO FLIGHT: 0.7825
        hdssm.put(2.1, 1875.0); // TIME TO FLIGHT: 0.9250
        hdssm.put(2.9, 1775.0); // TIME TO FLIGHT: 0.9625 //1975
        hdssm.put(3.5, 2225.0); // TIME TO FLIGHT: 1.0750
        hdssm.put(4.0, 2425.0); // TIME TO FLIGHT: 1.1750
        hdssm.put(5.0, 2825.0); // TIME TO FLIGHT: 1.2375

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

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        m_driverController.leftTrigger().whileTrue(
                drivetrain.applyRequest(() -> {
                    double vx = -joystick.getLeftY() * MaxSpeed * 0.1;
                    double vy = -joystick.getLeftX() * MaxSpeed * 0.1;

                    double omega = m_vision.getRotationToHub(drivetrain, vx, vy, hdftm) * MaxAngularRate;

                    // System.out.println("Distnace to HUB: " + m_vision.getDistanceToHub());

                    return drive
                            .withVelocityX(vx)
                            .withVelocityY(vy)
                            .withRotationalRate(omega)
                            .withRotationalDeadband(0);
                }));

        m_driverController.b().toggleOnTrue(new ToggleIntakePosition(m_intakepivot));
        m_driverController.rightTrigger().toggleOnTrue(new IntakePercent_Com(m_intakeroller, .45));
        m_driverController.a().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldCentric();
            drivetrain.getPigeon2().reset();
        }));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        m_operatorController.rightTrigger().toggleOnTrue(new HopperPercent_Com(m_hopper, 1));

        // m_intake.setDefaultCommand(new IntakeJoystick_Com(m_intake,
        // m_operatorController)); // Right stick
        // m_operatorController.rightBumper().toggleOnTrue(new
        // IntakePercent_Com(m_intake, .45));

        m_operatorController.y().onTrue(new ShooterPercent_Com(m_shooter, 0.0));

        m_operatorController.pov(0).onTrue(new SetFlywheelSpeed_Com(m_shooter, () -> 2750.0)); // to shoot from the corner

        m_operatorController.x().onTrue(
                new InstantCommand(() -> {
                    m_targetRPM += 25;
                    System.out.println("Right Bumper Pressed → Target RPM: " + m_targetRPM);
                }));

        /* Decrement target RPM */
        m_operatorController.b().onTrue(
                new InstantCommand(() -> {
                    m_targetRPM -= 25;
                    System.out.println("Left Bumper Pressed → Target RPM: " + m_targetRPM);
                }));

        // Changed from whileTrue to onTrue
        m_operatorController.a().onTrue(
                new SetFlywheelSpeed_Com(m_shooter, () -> m_targetRPM));

        m_driverController.y()
                .whileTrue(new ParallelCommandGroup(
                        new autoRangeFire_Com(
                                m_shooter,
                                m_vision,
                                m_driverController,
                                hdssm,
                                () -> -joystick.getLeftY() * MaxSpeed * 0.1, // vx lambda
                                () -> -joystick.getLeftX() * MaxSpeed * 0.1 // vy lambda
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(0.25),
                                new HopperPercent_Com(m_hopper, 1.0))));

        m_testingController.a().onTrue(new IncrementSpeedTesting_Com(m_shooter));
        m_testingController.x().onTrue(new IncrementSpeedUp_Com(m_shooter, 0.01));
        m_testingController.b().onTrue(new IncrementSpeedUp_Com(m_shooter, -0.01));

        //! all below are temp bc I don't like switching btw controllers when testing. Feel free to delete - Parker

        m_soloTestController.rightTrigger().whileTrue(new ParallelCommandGroup(//? mag dump while moving
                        new autoRangeFire_Com(
                                m_shooter,
                                m_vision,
                                m_soloTestController,
                                hdssm,
                                () -> -m_soloTestController.getLeftY() * MaxSpeed * 0.1, // vx lambda
                                () -> -m_soloTestController.getLeftX() * MaxSpeed * 0.1 // vy lambda
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(0.25),
                                new HopperPercent_Com(m_hopper, 1.0))));

        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-m_soloTestController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-m_soloTestController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-m_soloTestController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        m_soloTestController.leftTrigger().whileTrue(
                drivetrain.applyRequest(() -> {
                    double vx = -m_soloTestController.getLeftY() * MaxSpeed * 0.1;
                    double vy = -m_soloTestController.getLeftX() * MaxSpeed * 0.1;

                    double omega = m_vision.getRotationToHub(drivetrain, vx, vy, hdftm) * MaxAngularRate;
                    // System.out.println("Distnace to HUB: " + m_vision.getDistanceToHub());

                    return drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega).withRotationalDeadband(0);
                }));
        
        m_soloTestController.pov(0).onTrue(new SetFlywheelSpeed_Com(m_shooter, () -> 2750.0));
        m_soloTestController.pov(180).onTrue(new SetFlywheelSpeed_Com(m_shooter, () -> 0.0));

        m_soloTestController.leftBumper().toggleOnTrue(new ToggleIntakePosition(m_intakepivot));
        m_soloTestController.rightBumper().toggleOnTrue(new IntakePercent_Com(m_intakeroller, .45));

        m_soloTestController.a().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldCentric();
            drivetrain.getPigeon2().reset();
        }));
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
