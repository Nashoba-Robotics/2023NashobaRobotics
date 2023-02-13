package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Grabber;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class ManualExtensionCommand extends CommandBase {
    Joystick extendJoystick;
    double lastPos;
    boolean joystick0;

    public ManualExtensionCommand() {
        extendJoystick = new Joystick(5);
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ArmSubsystem.getInstance().set(0);
        SmartDashboard.putNumber("Arm Angle Man", 0);
        SmartDashboard.putNumber("Grab Speed Man", 0);
        SmartDashboard.putNumber("Grab Angle Man", 0);
    }

    @Override
    public void execute() {
        double y = extendJoystick.getX();
        y = Math.abs(y) < 0.1 ? 0 : (y-0.1)/0.9;    //Put deadzone in Constants
        if(y == 0){ // If there isn't any input, maintain the position
            ArmSubsystem.getInstance().holdArm();
            if(!joystick0){
                joystick0 = true;
                lastPos = ArmSubsystem.getInstance().getPos();
            }
            ArmSubsystem.getInstance().extendNU(lastPos);
        }
        else{
            ArmSubsystem.getInstance().set(-y*0.3);
            joystick0 = false;
        } 

        SmartDashboard.putNumber("Stator Current", ArmSubsystem.getInstance().getArmStatorCurrent());
        SmartDashboard.putNumber("Supply Current", ArmSubsystem.getInstance().getArmSupplyCurrent());
        SmartDashboard.putNumber("Extend NU", ArmSubsystem.getInstance().getPos());

        SmartDashboard.putNumber("Pivot Stator", ArmSubsystem.getInstance().getPivotStator());
        SmartDashboard.putNumber("Pivot Supply", ArmSubsystem.getInstance().getPivotSupply());

        double angle = SmartDashboard.getNumber("Arm Angle Man", 0);
        angle *= Constants.TAU/360;
        ArmSubsystem.getInstance().pivot(angle);

        double grabSpeed = SmartDashboard.getNumber("Grab Speed Man", 0);
        GrabberSubsystem.getInstance().set(grabSpeed);

        double grabNU = SmartDashboard.getNumber("Grab Angle Man", 0);
        GrabberSubsystem.getInstance().orientPos(grabNU);
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().set(0);
        lastPos = 103.2;
        ArmSubsystem.getInstance().reset();
        GrabberSubsystem.getInstance().orientPos(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
