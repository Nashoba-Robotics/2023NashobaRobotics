package frc.robot.commands.auto.systemcheck;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.CandleSubsystem.CandleState;

public class ConePrepCheck extends CommandBase{
    TargetLevel l;

    private double targetExtend;
    private double targetPivot;
    private double targetWrist;

    private boolean at20;
    private boolean resetPivot;

    public ConePrepCheck(TargetLevel l){
        this.l = l;

        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        switch(l){
            case HIGH:
                targetExtend = Constants.Arm.HIGH_EXTEND_NU;
                targetPivot = Constants.Arm.HIGH_FRONT_ANGLE;
                targetWrist = Constants.Grabber.PREP_CONE_FRONT_NU;
                break;
            case MID:
                targetExtend = Constants.Arm.MID_EXTEND_NU;
                targetPivot = Constants.Arm.MID_ANGLE;
                targetWrist = Constants.Grabber.MID_ANGLE;
                break;
            case LOW:
                targetExtend = Constants.Arm.LOW_EXTEND_NU;
                targetPivot = Constants.Arm.LOW_ANGLE;
                targetWrist = Constants.Grabber.LOW_ANGLE;
                break;
        }
        targetPivot *= -1;  //Want to score backside to keep everything consistent

        at20 = Math.abs(ArmSubsystem.getInstance().getEncoderAngle()-22) < 2;
        resetPivot = false;

        ArmSubsystem.getInstance().resetPivotNU();
    }

    @Override
    public void execute() {
        if(!at20){
            ArmSubsystem.getInstance().setPivotCruiseVelocity(30_000);
            ArmSubsystem.getInstance().setPivotAcceleration(20_000);

            ArmSubsystem.getInstance().pivot(22 * Constants.TAU/360);
        }
        else{
            ArmSubsystem.getInstance().setDefaultCruiseVelocity();
            ArmSubsystem.getInstance().setDefaultAcceleration();
            ArmSubsystem.getInstance().extendNU(targetExtend);
            ArmSubsystem.getInstance().pivot(targetPivot);
            GrabberSubsystem.getInstance().orientPos(targetWrist);

            if(!resetPivot && Math.abs(targetPivot - ArmSubsystem.getInstance().getAngle()) < 1 * Constants.TAU/360
            && ArmSubsystem.getInstance().getPivotSpeed() < 3){
                resetPivot = true;
                ArmSubsystem.getInstance().resetPivotNU();
            }

            if(resetPivot && ArmSubsystem.getInstance().getPivotSpeed() < 3){
                if(Math.abs(ArmSubsystem.getInstance().getEncoderAngle() - targetPivot * 360/Constants.TAU) < 1){
                    CandleSubsystem.getInstance().set(CandleState.SYSTEM_GOOD);
                }
                else if(Math.abs(ArmSubsystem.getInstance().getEncoderAngle() - targetPivot * 360/Constants.TAU) < 5){
                    CandleSubsystem.getInstance().set(CandleState.PARTIAL_CHECK_1);
                }
            }
        }
        
    }

    @Override
    public boolean isFinished() {
        //Later on, can change this to a button push
        return resetPivot && Math.abs(ArmSubsystem.getInstance().getEncoderAngle() - targetPivot * 360/Constants.TAU) < 5;
    }
}
