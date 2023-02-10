package frc.robot.commands.test;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmTestCommand extends CommandBase{
    private ShuffleboardTab armTab = Shuffleboard.getTab("Arm Testing");

    @Override
    public void initialize() {
        ArmSubsystem.getInstance().zeroPivot();

        // Shuffleboard.getTab("Arm Testing");
        // armTab.add("Arm Speed", 0);
        // armTab.add("Pivot Speed", 0);
        // armTab.add("Pivot Angle", 0);

        SmartDashboard.putNumber("Arm Speed", 0);
        SmartDashboard.putNumber("Pivot Speed", 0);
        SmartDashboard.putNumber("Pivot Angle", 0);
    }

    //TODO: See if NU positive goes in the correct direction
    @Override
    public void execute() {
        // TODO: Try .addPersistent()
        //armTab.add("Arm NU", ArmSubsystem.getInstance().getPos());
        //armTab.add("Arm Len", ArmSubsystem.getInstance().getLength());
        //armTab.add("Limit Switch Hit?", ArmSubsystem.getInstance().extended());

        // Do I need to put this into the tab? Or can I just do this?
        // double speed = armTab.add("Arm Speed", 0).getEntry().getDouble(0);
        // double speed = SmartDashboard.getNumber("Arm Speed", 0);
        // ArmSubsystem.getInstance().set(speed);

        SmartDashboard.putNumber("Pivot NU 1", ArmSubsystem.getInstance().getPivotPos(1));
        SmartDashboard.putNumber("Pivot NU 2", ArmSubsystem.getInstance().getPivotPos(2)); 

        SmartDashboard.putNumber("Pivot Angle 1", ArmSubsystem.getInstance().getPivotAngleDeg(1));
        SmartDashboard.putNumber("Pivot Angle 2", ArmSubsystem.getInstance().getPivotAngleDeg(2));

        //double pivotSpeed = armTab.add("Pivot Speed", 0).getEntry().getDouble(0);
        // double pivotSpeed = SmartDashboard.getNumber("Pivot Speed", 0);
        // if(Math.abs(pivotSpeed) > 0.3) pivotSpeed = 0.15;    //Don't want to accidentally kill someone
        // ArmSubsystem.getInstance().setPivot(pivotSpeed);

        // double angle = armTab.add("Pivot Angle", 0).getEntry().getDouble(0);
        double angle = SmartDashboard.getNumber("Pivot Angle", 0);
        SmartDashboard.putNumber("Value", angle);
        angle *= Constants.TAU/360;
        ArmSubsystem.getInstance().pivot(angle);
    }

    @Override
    public void end(boolean interrupted) {
        //ArmSubsystem.getInstance().set(0);
        ArmSubsystem.getInstance().setPivot(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
