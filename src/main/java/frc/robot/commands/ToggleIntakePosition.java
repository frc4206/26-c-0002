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
    private boolean stow_only;

    public ToggleIntakePosition(IntakePivotSub intake, boolean stow_only) {
        this.intake = intake;
        this.stow_only = stow_only;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Decide what the target should be
        if (stow_only) {
            // Only stow if currently deployed
            if (isDeployed) {
                target = STOWED;
                intake.setPositionOfIntakePivot(target);
                System.out.println("STOW_ONLY: Moving to " + target);
                System.out.println("MOTOR IS AT: " + intake.intakePivotMotor.getPosition());
                isDeployed = false; // now it’s stowed
            } else {
                // Already stowed, do nothing
                System.out.println("STOW_ONLY: Already stowed, skipping.");
            }
        } else {
            // Normal toggle: deploy if stowed, stow if deployed
            target = isDeployed ? STOWED : DEPLOYED;
            intake.setPositionOfIntakePivot(target);
            System.out.println("TOGGLE: Moving to " + target);
            System.out.println("MOTOR IS AT: " + intake.intakePivotMotor.getPosition());
            isDeployed = !isDeployed;
        }
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