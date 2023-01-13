package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SwerveDriveCommand extends CommandBase {
    private Joystick leftJoystick;
    private Joystick rightJoystick;

    public SwerveDriveCommand() {
        addRequirements(SwerveDriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
            rightJoystick = new Joystick(0);
            leftJoystick = new Joystick(1);
    }

    @Override
    public void execute() {
            double[] leftJoystickValues = {leftJoystick.getX(), leftJoystick.getY()};
            double rightJoystickValue = rightJoystick.getX();
            SwerveDriveSubsystem.getInstance().set(leftJoystickValues[0], leftJoystickValues[1], rightJoystickValue);
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