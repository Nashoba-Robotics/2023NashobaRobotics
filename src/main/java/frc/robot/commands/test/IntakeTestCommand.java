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
        Tabs.Intake.resetAll();

        ArmSubsystem.getInstance().setPivotCruiseVelocity(20_000);
        ArmSubsystem.getInstance().setPivotAcceleration(20_000);

        ArmSubsystem.getInstance().resetPivotNU();
    }

    @Override
    public void execute() {
        double armAngle = Tabs.Intake.getPivotAngle();  //Input in degrees. Calculates in radians
        ArmSubsystem.getInstance().pivot(armAngle);

        double extendNU = Tabs.Intake.getExtendNU();
        ArmSubsystem.getInstance().extendNU(extendNU);

        double wristNU = Tabs.Intake.getOrienterNU();
        GrabberSubsystem.getInstance().orientPos(wristNU);

        double wristSpeed = Tabs.Intake.getGrabSpeed();
        GrabberSubsystem.getInstance().set(wristSpeed, wristSpeed);

        Tabs.Intake.displayExtendNU(ArmSubsystem.getInstance().getPos());
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
        Tabs.Intake.displayTopStator(GrabberSubsystem.getInstance().getTopGrabCurrent());
        Tabs.Intake.displayBotStator(GrabberSubsystem.getInstance().getBotGrabCurrent());
        Tabs.Intake.displayEncoder(ArmSubsystem.getInstance().getEncoderAngle());

        Tabs.Intake.displayMM(ArmSubsystem.getInstance().getExtendNU());
        Tabs.Intake.displayPivotOutput(ArmSubsystem.getInstance().getPivotOutput());
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