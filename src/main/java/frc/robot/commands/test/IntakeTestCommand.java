package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.LogManager;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class IntakeTestCommand extends CommandBase{
    public IntakeTestCommand(){
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ArmSubsystem.getInstance().zeroPivotSensor();
        ArmSubsystem.getInstance().zeroArmSensor();
        GrabberSubsystem.getInstance().zeroWrist();

        // ArmSubsystem.getInstance().extend(0);   //Added this to keep the arm in place as the pivot moves
    
        SmartDashboard.putNumber("Arm Angle", 0);
        SmartDashboard.putNumber("SetWristNU", 0);

        SmartDashboard.putNumber("Intake Speed", 0);
        SmartDashboard.putNumber("Extend NU", 0);
    }

    @Override
    public void execute() {
        double wristAngle = SmartDashboard.getNumber("SetWristNU", 0); //In NU
        // wristAngle *= Constants.TAU/360;    //For some reason, 1200 seemed best for intaking, and 1500 seemed good for scoring
        GrabberSubsystem.getInstance().orientPos(wristAngle);

        double wristSpeed = SmartDashboard.getNumber("Intake Speed", 0);
        // GrabberSubsystem.getInstance().set(wristSpeed, -wristSpeed); //Used -0.7 to intake
        GrabberSubsystem.getInstance().set(wristSpeed); //Used -0.7 to intake

        double pivotAngle = SmartDashboard.getNumber("Arm Angle", 0);   //In degrees
        pivotAngle *= Constants.TAU/360; 
        ArmSubsystem.getInstance().pivot(pivotAngle);   //107 degrees seemed good for intaking

        double extendNU = SmartDashboard.getNumber("Extend NU", 0);

        ArmSubsystem.getInstance().extendNU(extendNU);

        SmartDashboard.putNumber("Pivot NU", ArmSubsystem.getInstance().getPivotPos(1));
        SmartDashboard.putNumber("Pivot Angle Actual", ArmSubsystem.getInstance().getPivotAngleDeg(1));

        SmartDashboard.putNumber("Wrist NU", GrabberSubsystem.getInstance().getPosition());
        SmartDashboard.putNumber("Actual Wrist Angle", GrabberSubsystem.getInstance().getOrientation() * 360 / Constants.TAU);

        // LogManager.appendToLog(wristAngle, "Wrist:/SetState");
        // LogManager.appendToLog(GrabberSubsystem.getInstance().getOrientPos(), "Wrist:/ActualState");
        
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().stop();
        GrabberSubsystem.getInstance().stop();
        ArmSubsystem.getInstance().setPivot(0);
        ArmSubsystem.getInstance().reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}