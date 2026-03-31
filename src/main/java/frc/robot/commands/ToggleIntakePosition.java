package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivotSub;

public class ToggleIntakePosition extends Command {
    private final IntakePivotSub intake;
    private static boolean isDeployed = false; // static so it persists between command instances

    private static final double STOWED = 0.0;
    private static final double DEPLOYED = -3.1;
    private static final double TOLERANCE_STOWED = 0.3;
    private static final double TOLERANCE_DEPLOYED = 0.3;

    private double target;

    public ToggleIntakePosition(IntakePivotSub intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        target = isDeployed ? STOWED : DEPLOYED;
        isDeployed = !isDeployed;
        intake.setPositionOfIntakePivot(target);

        System.out.println("Moving to: " + target);
        System.out.println("MOTOR IS AT: " + intake.intakePivotMotor.getPosition());
    }

    @Override
    public boolean isFinished() {
        double current = intake.intakePivotMotor.getPosition().getValueAsDouble();
        // System.out.println("current position: " + current);
        double tolerance = (target == STOWED) ? TOLERANCE_STOWED : TOLERANCE_DEPLOYED;
        // boolean result = (target + current) < tolerance;
        double distance_to_go_to_target = target - current;
        if (Math.abs(distance_to_go_to_target) > tolerance) {
            return false;
        } else {
            return true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            isDeployed = !isDeployed;
        }
        intake.intakePivotMotor.set(0); // or set motor output to 0
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}