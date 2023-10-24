package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.DriveSpeedCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TJTestCommand extends CommandBase{
    double speed = 0;
    public TJTestCommand(){
        addRequirements(SwerveDriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SwerveDriveSubsystem.getInstance().setGyro(0);
        SmartDashboard.putNumber("TJ Test Spd", 0);
    }

    @Override
    public void execute() {
        speed = SmartDashboard.getNumber("TJ Test Spd", 0);
        SwerveDriveSubsystem.getInstance().set(speed, 0, 0);

        for(int i = 0; i < 4; i++){
            SmartDashboard.putNumber("TJ Test Move Current " + i, SwerveDriveSubsystem.getInstance().getMoveStator(i));
            SmartDashboard.putNumber("TJ Test Turn Current " + i, SwerveDriveSubsystem.getInstance().getTurnStator(i));

        }
    }

    @Override
    public void end(boolean interrupted) {
        SwerveDriveSubsystem.getInstance().set(0, 0, 0);
    }
}
