package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;

public class RunFlywheelVoltage extends Command {

    private final ShooterSub shooter;
    private final double testVoltage;

    public RunFlywheelVoltage(ShooterSub shooter, double volts) {
        this.shooter = shooter;
        this.testVoltage = volts;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setFlywheelVoltage(testVoltage);
        // System.out.println("SENDING VOLTAGS OMAG");
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}