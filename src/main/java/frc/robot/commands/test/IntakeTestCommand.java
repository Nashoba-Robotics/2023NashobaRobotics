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

        // ArmSubsystem.getInstance().setPivotCruiseVelocity(20_000);
        // ArmSubsystem.getInstance().setPivotAcceleration(20_000);

        // ArmSubsystem.getInstance().resetPivotNU();
    }

    @Override
    public void execute() {
        double armAngle = Tabs.Intake.getPivotAngle();  //Input in degrees. Calculates in radians
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

        Tabs.Intake.displayPivotAngle(ArmSubsystem.getInstance().getAngle());
        Tabs.Intake.displayPivotCurrent(
            ArmSubsystem.getInstance().getPivotStatorCurrent(), 
            ArmSubsystem.getInstance().getPivotSupplyCurrent()
        );

        Tabs.Intake.displayOrienterNU(GrabberSubsystem.getInstance().getOrientPos());
        Tabs.Intake.displayTopStator(GrabberSubsystem.getInstance().getGrabberCurrent());
        Tabs.Intake.displayEncoder(ArmSubsystem.getInstance().getEncoderAngle());

        Tabs.Intake.displayMM(ArmSubsystem.getInstance().getExtendNU());
        Tabs.Intake.displayPivotOutput(ArmSubsystem.getInstance().getPivotOutput());

        // ArmSubsystem.getInstance().setPivotP(Tabs.Intake.pivotP.getDouble(Constants.Arm.PIVOT_KP_1));
        // ArmSubsystem.getInstance().setPivotI(Tabs.Intake.pivotI.getDouble(Constants.Arm.PIVOT_KI_1));
        // ArmSubsystem.getInstance().setPivotD(Tabs.Intake.pivotD.getDouble(Constants.Arm.PIVOT_KD_1));

        // ArmSubsystem.getInstance().setExtendP(Tabs.Intake.extendP.getDouble(Constants.Arm.ARM_KP));
        // ArmSubsystem.getInstance().setExtendI(Tabs.Intake.extendI.getDouble(Constants.Arm.ARM_KI));
        // ArmSubsystem.getInstance().setExtendD(Tabs.Intake.extendD.getDouble(Constants.Arm.ARM_KD));

        // ArmSubsystem.getInstance().setPivotAcceleration(Tabs.Intake.pivotAcceleration.getDouble(20_000));
        // ArmSubsystem.getInstance().setPivotCruiseVelocity(Tabs.Intake.pivotCruiseVelocity.getDouble(20_000));

        // ArmSubsystem.getInstance().setExtendAcceleration(Tabs.Intake.extendAcceleration.getDouble(20_000));
        // ArmSubsystem.getInstance().setExtendCruiseVelocity(Tabs.Intake.extendCruiseVelocity.getDouble(20_000));
    }

    @Override
    public void end(boolean interrupted) {
        // ArmSubsystem.getInstance().stop();
        // GrabberSubsystem.getInstance().stop();
        // ArmSubsystem.getInstance().setPivot(0);
        // ArmSubsystem.getInstance().reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}