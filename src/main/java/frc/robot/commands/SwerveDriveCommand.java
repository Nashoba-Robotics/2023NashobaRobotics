package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.JoystickSubsytem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SwerveDriveCommand extends CommandBase {

    public SwerveDriveCommand() {
        addRequirements(
            SwerveDriveSubsystem.getInstance(),
            JoystickSubsytem.getInstance()
        );
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double multiplier = 1;

        SwerveDriveSubsystem.getInstance().set(
            JoystickSubsytem.getInstance().getLeftJoystickValues().shape(
                Constants.Joystick.MOVE_DEAD_ZONE,
                Constants.Joystick.MOVE_SENSITIVITY
            ).multiply(multiplier).swap().applyAngleDeadzone(10 * Constants.TAU/360),
            JoystickSubsytem.getInstance().getRightJoystickValues().shape(
                Constants.Joystick.TURN_DEAD_ZONE,
                Constants.Joystick.TURN_SENSITIVITY
            ).x,
            false
            );
    }

    @Override
    public void end(boolean interrupted) {
        // SwerveDriveSubsystem.getInstance().set(0, 0, 0);
    }
 
    @Override
    public boolean isFinished() {
        return false;
    }

}