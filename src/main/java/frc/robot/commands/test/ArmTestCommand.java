package frc.robot.commands.test;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmTestCommand extends CommandBase{
    private ShuffleboardTab armTab = Shuffleboard.getTab("Arm Testing");

    @Override
    public void initialize() {
        ArmSubsystem.getInstance().zeroPivot();

        Shuffleboard.getTab("Arm Testing");
    }

    //TODO: See if NU positive goes in the correct direction
    @Override
    public void execute() {
        // TODO: Try .addPersistent()
        armTab.add("Arm NU", ArmSubsystem.getInstance().getPos());
        armTab.add("Arm Len", ArmSubsystem.getInstance().getLength());
        armTab.add("Limit Switch Hit?", ArmSubsystem.getInstance().extended());

        // Do I need to put this into the tab? Or can I just do this?
        double speed = armTab.add("Arm Speed", 0).getEntry().getDouble(0);
        ArmSubsystem.getInstance().set(speed);

        armTab.add("Pivot NU 1", ArmSubsystem.getInstance().getPivotPos(1));
        armTab.add("Pivot NU 2", ArmSubsystem.getInstance().getPivotPos(2)); 

        armTab.add("Pivot Angle 1", ArmSubsystem.getInstance().getPivotAngleDeg(1));
        armTab.add("Pivot Angle 2", ArmSubsystem.getInstance().getPivotAngleDeg(2));

        double pivotSpeed = armTab.add("Pivot Speed", 0).getEntry().getDouble(0);
        ArmSubsystem.getInstance().setPivot(pivotSpeed);

        double angle = armTab.add("Pivot Angle", 0).getEntry().getDouble(0);
        angle *= Constants.TAU/360;
        ArmSubsystem.getInstance().pivot(angle);
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().set(0);
        ArmSubsystem.getInstance().setPivot(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
