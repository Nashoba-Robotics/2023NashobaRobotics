package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class IntakeCommand extends CommandBase {
    double armAngle = 112 * Constants.TAU/360;  //112
    double wristNU = 4;
    double wristAngle = 0;

    public IntakeCommand(double armAngle, double wristAngle) {
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());

        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
    }

    public IntakeCommand(){
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        // Extend is TEMP to test at the same distance
        ArmSubsystem.getInstance().extend(0);
        ArmSubsystem.getInstance().pivot(armAngle);
        GrabberSubsystem.getInstance().orientPos(wristNU);
        //GrabberSubsystem.getInstance().orient(wristAngle);
    }

    @Override
    public void execute() {
        GrabberSubsystem.getInstance().intake();
        SmartDashboard.putNumber("Grabber Current", GrabberSubsystem.getInstance().getCurrent());
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().pivot(0);
        GrabberSubsystem.getInstance().orient(0);
        GrabberSubsystem.getInstance().set(-0.1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
