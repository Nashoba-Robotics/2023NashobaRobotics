package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Tabs;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class IntakeTestCommand extends CommandBase{
    public IntakeTestCommand(){
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        GrabberSubsystem.getInstance().setCurrentLimit(false);
        Tabs.Intake.resetAll();

        ArmSubsystem.getInstance().setDefaultCruiseVelocity();
        ArmSubsystem.getInstance().setDefaultAcceleration();

        // ArmSubsystem.getInstance().resetPivotNU();
    }

    @Override
    public void execute() {
        double armAngle = Tabs.Intake.getPivotAngle();  //Input in degrees
        ArmSubsystem.getInstance().pivot(armAngle);

        double extendNU = Tabs.Intake.getExtendNU();
        ArmSubsystem.getInstance().extendNU(extendNU);

        double wristNU = Tabs.Intake.getOrienterNU();
        GrabberSubsystem.getInstance().orientPos(wristNU);

        double grabberSpeed = Tabs.Intake.getGrabSpeed();
        GrabberSubsystem.getInstance().set(grabberSpeed);

        Tabs.Intake.displayExtendNU(ArmSubsystem.getInstance().getExtendNU());
        Tabs.Intake.displayExtendCurrent(
            ArmSubsystem.getInstance().getArmStatorCurrent(), 
            ArmSubsystem.getInstance().getArmSupplyCurrent()
        );

        Tabs.Intake.displayPivotAngle(ArmSubsystem.getInstance().getPivotDeg());
        Tabs.Intake.displayPivotCurrent(
            ArmSubsystem.getInstance().getPivotStatorCurrent(), 
            ArmSubsystem.getInstance().getPivotSupplyCurrent()
        );

        Tabs.Intake.displayOrienterNU(GrabberSubsystem.getInstance().getOrientPos());
        Tabs.Intake.displayTopStator(GrabberSubsystem.getInstance().getGrabberCurrent());
        Tabs.Intake.displayEncoder(ArmSubsystem.getInstance().getEncoderDeg());

        // Tabs.Intake.displayMM(ArmSubsystem.getInstance().getExtendNU());
        Tabs.Intake.displayPivotOutput(ArmSubsystem.getInstance().getTest1());
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().stop();
        GrabberSubsystem.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}