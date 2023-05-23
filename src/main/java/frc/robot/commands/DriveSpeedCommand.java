package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Tabs;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveSpeedCommand extends CommandBase{
    public DriveSpeedCommand(){
        addRequirements(SwerveDriveSubsystem.getInstance(), ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    } 

    @Override
    public void initialize() {
        Tabs.DriveTest.displayMove();
        SwerveDriveSubsystem.getInstance().zeroYaw();

        ArmSubsystem.getInstance().resetPivotNU();
        ArmSubsystem.getInstance().extendNU(3000);
    }

    @Override
    public void execute() {
        double speed = Tabs.DriveTest.getDriveSpeed();
        if(Tabs.DriveTest.move()){
            SwerveDriveSubsystem.getInstance().set(speed, 0, 0);
            GrabberSubsystem.getInstance().set(-Tabs.DriveTest.getGrabberSpeed());
        }
        else{
            SwerveDriveSubsystem.getInstance().set(0, 0, 0);
            GrabberSubsystem.getInstance().set(0);
        }
        // Tabs.DriveTest.displayActualSpeed(SwerveDriveSubsystem.getInstance().getXVelocity());

        ArmSubsystem.getInstance().pivot(
            Tabs.DriveTest.getPivotAngle()*Constants.TAU/360
        );

        Tabs.DriveTest.displayActualAngle(ArmSubsystem.getInstance().getPivotRad()*360/Constants.TAU);

        GrabberSubsystem.getInstance().orientPos(
            Tabs.DriveTest.grabberAngle()
        );

        Tabs.DriveTest.displayGrabberCurrent(GrabberSubsystem.getInstance().getGrabberCurrent());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
