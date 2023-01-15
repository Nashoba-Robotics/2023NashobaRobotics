package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TestCommand extends CommandBase {

    public TestCommand() {
        addRequirements(SwerveDriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        // for(double angle : SwerveDriveSubsystem.getInstance().getModAngles()) {
        //     System.out.print("Encoder Angle:" + angle + " ");
        // }
        // System.out.println();
        // for(double angle : SwerveDriveSubsystem.getInstance().getAnglePositions()) {
        //     System.out.print("Motor Angle:" + angle + " ");
        // }
        // System.out.println();
    }

    @Override
    public void execute() {
        SwerveDriveSubsystem.getInstance().set(0, 0.2, 0);
    }

    @Override
    public void end(boolean interrupted) {
        SwerveDriveSubsystem.getInstance().setDirectly(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
