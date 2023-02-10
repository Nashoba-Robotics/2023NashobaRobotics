package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SwerveDriveTestCommand extends CommandBase {

    public SwerveDriveTestCommand() {
        addRequirements(new SubsystemBase[]{
            SwerveDriveSubsystem.getInstance(),
        });
    }

    @Override
    public void initialize() {
        SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/2);
        SwerveDriveSubsystem.getInstance().set(0, 0, 0);
        SmartDashboard.putNumber("kiddy mode", 0);
        SwerveDriveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        SmartDashboard.putNumber("XSet", 0);
        SmartDashboard.putNumber("YSet", 0);
        SmartDashboard.putNumber("OSet", 0);
    }

    @Override
    public void execute() {
            SwerveDriveSubsystem.getInstance().set(
                SmartDashboard.getNumber("XSet", 0),
                SmartDashboard.getNumber("YSet", 0),
                SmartDashboard.getNumber("OSet", 0)
                );

                for(int i = 1; i <= 4; i++){
                    SmartDashboard.putNumber("Mod " + i + " Angle", SwerveDriveSubsystem.getInstance().getModAngles()[i-1]);
                }
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