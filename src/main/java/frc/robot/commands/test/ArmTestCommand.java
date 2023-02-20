package frc.robot.commands.test;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Tabs;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSubsystem;

public class ArmTestCommand extends CommandBase{
   // GenericEntry armSpeed = Tabs.Arm.add("Arm Speed", 0).getEntry();
    // GenericEntry armPos = Tabs.Arm.add("Arm Pos", 0).getEntry();

    // GenericEntry pivotSpeed = Tabs.Arm.add("Pivot Angle", 0).getEntry();
    // GenericEntry armNU = Tabs.Arm.add("Arm NU", 0).getEntry();
    // GenericEntry armLen = Tabs.Arm.add("Arm Len", 0).getEntry();
    // GenericEntry armStator = Tabs.Arm.add("Extend Stator", 0).getEntry();
    // GenericEntry armSupply = Tabs.Arm.add("Extend Supply", 0).getEntry();

    @Override
    public void initialize() {
        ArmSubsystem.getInstance().zeroPivot();

        // Shuffleboard.getTab("Arm Testing");
        // armTab.add("Arm Speed", 0);
        // armTab.add("Pivot Speed", 0);
        // armTab.add("Pivot Angle", 0);

        // SmartDashboard.putNumber("Arm Speed", 0);
        // SmartDashboard.putNumber("Pivot Speed", 0);
        // ArmSubsystem.getInstance().zeroArm();
        SmartDashboard.putNumber("Arm Set Pos", 0);
        SmartDashboard.putNumber("SetArmNU", 0);
    }

    //TODO: See if NU positive goes in the correct direction
    @Override
    public void execute() {
        // TODO: Try .addPersistent()
        // armNU.setDouble(ArmSubsystem.getInstance().getPos());
        // armLen.setDouble( ArmSubsystem.getInstance().getLength());
        // armStator.setDouble(ArmSubsystem.getInstance().getStatorCurrent());
        // armSupply.setDouble(ArmSubsystem.getInstance().getSupplyCurrent());
        //armTab.add("Limit Switch Hit?", ArmSubsystem.getInstance().extended());

        //double speed = armSpeed.getDouble(0);
        //double speed = SmartDashboard.getNumber("Arm Speed", 0);
        //ArmSubsystem.getInstance().set(speed);

        // double pos = armPos.getDouble(0);
        // ArmSubsystem.getInstance().extend(pos); // In meters

        // Tabs.Arm.add("Pivot NU 1", ArmSubsystem.getInstance().getPivotPos(1));
        // Tabs.Arm.add("Pivot NU 2", ArmSubsystem.getInstance().getPivotPos(2)); 

        // Tabs.Arm.add("Avg Stator Current", ArmSubsystem.getInstance().getPivotStator());
        // Tabs.Arm.add("Avg Supply Current", ArmSubsystem.getInstance().getPivotSupply());

        // armTab.add("Pivot Angle 1", ArmSubsystem.getInstance().getPivotAngleDeg(1));
        // armTab.add("Pivot Angle 2", ArmSubsystem.getInstance().getPivotAngleDeg(2));

        //double pivotSpeed = armTab.add("Pivot Speed", 0).getEntry().getDouble(0);
        // double pivotSpeed = SmartDashboard.getNumber("Pivot Speed", 0);
        // if(Math.abs(pivotSpeed) > 0.3) pivotSpeed = 0.15;    //Don't want to accidentally kill someone
        // ArmSubsystem.getInstance().setPivot(pivotSpeed);

        // double angle = pivotSpeed.getDouble(0);
        // angle *= Constants.TAU/360;
        // ArmSubsystem.getInstance().pivot(angle);

        double pos = SmartDashboard.getNumber("SetArmNU", 0);    //NU
        ArmSubsystem.getInstance().extendNU(pos);

        double angle = SmartDashboard.getNumber("Arm Angle", 0);    //Degrees
        angle *= Constants.TAU/360;
        ArmSubsystem.getInstance().pivot(angle);

        SmartDashboard.putNumber("Extend NU", ArmSubsystem.getInstance().getPos());
        SmartDashboard.putNumber("Pivot NU", ArmSubsystem.getInstance().getPivotPos(1));

        SmartDashboard.putNumber("Extend Stator", ArmSubsystem.getInstance().getArmStatorCurrent());
        SmartDashboard.putNumber("Extend Supply", ArmSubsystem.getInstance().getArmSupplyCurrent());

        SmartDashboard.putNumber("Pivot Stator", ArmSubsystem.getInstance().getPivotStator());
        SmartDashboard.putNumber("Pivot Supply", ArmSubsystem.getInstance().getPivotSupply());
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
