package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/2);
        SwerveDriveSubsystem.getInstance().set(0, 0, 0);
        SmartDashboard.putNumber("kiddy mode", 0);
        SwerveDriveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }

    @Override
    public void execute() {
        double multiplier = (int)SmartDashboard.getNumber("kiddy mode", 0)==1 ? 0.3 : 1;

            SwerveDriveSubsystem.getInstance().set(
                JoystickSubsytem.getInstance().getLeftJoystickValues().shape(
                    Constants.Joystick.MOVE_DEAD_ZONE,
                    Constants.Joystick.MOVE_SENSITIVITY
                ).multiply(multiplier),
                JoystickSubsytem.getInstance().getRightJoystickValues().shape(
                    Constants.Joystick.TURN_DEAD_ZONE,
                    Constants.Joystick.TURN_SENSITIVITY
                ).multiply(multiplier).x,
                true
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