package frc.robot.commands.test;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem.TargetType;

/*
Assuming the robot is alread oriented correctly with angle, 
the robot will move horizontally until the April tag is in the middle (centered)

Uses PID control rather than a snapshot
*/
public class CameraCenterCommand extends CommandBase{

    public CameraCenterCommand(){
        addRequirements(SwerveDriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
