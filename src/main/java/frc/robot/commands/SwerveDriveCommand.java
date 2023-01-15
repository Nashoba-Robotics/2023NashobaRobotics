package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.JoystickSubsytem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SwerveDriveCommand extends CommandBase {
    public SwerveDriveCommand() {
        addRequirements(new SubsystemBase[]{
            SwerveDriveSubsystem.getInstance(),
            JoystickSubsytem.getInstance(),
        });
    }

    @Override
    public void initialize() {
        SwerveDriveSubsystem.getInstance().set(0, 0, 0);
    }

    @Override
    public void execute() {
            SwerveDriveSubsystem.getInstance().set(
                JoystickSubsytem.getInstance().getLeftJoystickValues().shape(
                    Constants.Joystick.MOVE_DEAD_ZONE,
                    Constants.Joystick.MOVE_SENSITIVITY
                ),
                JoystickSubsytem.getInstance().getRightJoystickValues().shape(
                    Constants.Joystick.TURN_DEAD_ZONE,
                    Constants.Joystick.TURN_SENSITIVITY
                ).x
                );
    }

    @Override
    public void end(boolean interrupted) {
        SwerveDriveSubsystem.getInstance().set(0, 0, 0);
    }
 
    @Override
    public boolean isFinished() {
        return false;
    }

}