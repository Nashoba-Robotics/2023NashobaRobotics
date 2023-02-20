package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.LogManager;
import frc.robot.Tabs;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class IntakeTestCommand extends CommandBase{
    public IntakeTestCommand(){
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        // ArmSubsystem.getInstance().zeroPivot();
        // ArmSubsystem.getInstance().zeroArm();
        // GrabberSubsystem.getInstance().zeroWrist();

        // ArmSubsystem.getInstance().extend(0);   //Added this to keep the arm in place as the pivot moves
    }

    @Override
    public void execute() {
        // wristAngle *= Constants.TAU/360;    //For some reason, 1200 seemed best for intaking, and 1500 seemed good for scoring
        double wristAngle = Tabs.Intake.getOrienterNU();   //In NU
        GrabberSubsystem.getInstance().orientPos(wristAngle);

        // double wristSpeed = SmartDashboard.getNumber("Intake Speed", 0);
        double wristSpeed = Tabs.Intake.getGrabSpeed();
        // GrabberSubsystem.getInstance().set(wristSpeed, -wristSpeed); //Used -0.7 to intake
        GrabberSubsystem.getInstance().set(wristSpeed); //Used -0.7 to intake

        // double pivotAngle = SmartDashboard.getNumber("Arm Angle", 0);   //In degrees
        double pivotAngle = Tabs.Intake.getPivotAngle();    //In degrees
        pivotAngle *= Constants.TAU/360; 
        ArmSubsystem.getInstance().pivot(pivotAngle);   //107 degrees seemed good for intaking

        // double extendNU = SmartDashboard.getNumber("Extend NU", 0);
        double extendNU = Tabs.Intake.getExtendNU();   //NU
        ArmSubsystem.getInstance().extendNU(extendNU);

        Tabs.Intake.displayPivotNU(ArmSubsystem.getInstance().getPivotPos(1));
        Tabs.Intake.displayPivotAngle(ArmSubsystem.getInstance().getPivotAngleDeg(1));
        Tabs.Intake.displayOrienterNU(GrabberSubsystem.getInstance().getPosition());

        Tabs.Intake.displayExtendCurrent(ArmSubsystem.getInstance().getArmStatorCurrent(), ArmSubsystem.getInstance().getArmSupplyCurrent());
        Tabs.Intake.displayPivotCurrent(ArmSubsystem.getInstance().getPivotStator(), ArmSubsystem.getInstance().getPivotSupply());

        Tabs.Intake.displayOrientStator(GrabberSubsystem.getInstance().getCurrent());   //Output current to the motor

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
