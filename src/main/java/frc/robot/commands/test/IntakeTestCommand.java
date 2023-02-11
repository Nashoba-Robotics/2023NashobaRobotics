package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class IntakeTestCommand extends CommandBase{
    public IntakeTestCommand(){
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ArmSubsystem.getInstance().zeroPivot();
        GrabberSubsystem.getInstance().zeroWrist();
        SmartDashboard.putNumber("Arm Angle", 0);
        SmartDashboard.putNumber("Wrist Angle", 0);

        SmartDashboard.putNumber("Intake Speed", 0);
    }

    @Override
    public void execute() {
        double wristAngle = SmartDashboard.getNumber("Wrist Angle", 0); //In degrees
        wristAngle *= Constants.TAU/360;    //For some reason, 1200 seemed best for intaking, and 1500 seemed good for scoring
        GrabberSubsystem.getInstance().orient(wristAngle);

        double wristSpeed = SmartDashboard.getNumber("Intake Speed", 0);
        GrabberSubsystem.getInstance().set(wristSpeed); //Used -0.7 to intake

        double pivotAngle = SmartDashboard.getNumber("Arm Angle", 0);   //In degrees
        pivotAngle *= Constants.TAU/360; 
        ArmSubsystem.getInstance().pivot(pivotAngle);   //107 degrees seemed good for intaking

        SmartDashboard.putNumber("Pivot NU", ArmSubsystem.getInstance().getPivotPos(1));
        SmartDashboard.putNumber("Pivot Angle Actual", ArmSubsystem.getInstance().getPivotAngleDeg(1));
    
        
    }

    @Override
    public void end(boolean interrupted) {
        GrabberSubsystem.getInstance().stop();
        ArmSubsystem.getInstance().setPivot(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
