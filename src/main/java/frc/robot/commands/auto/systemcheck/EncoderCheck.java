package frc.robot.commands.auto.systemcheck;

import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.CandleSubsystem.CandleState;

public class EncoderCheck extends CommandBase{
    private boolean start;
    private ErrorCode e;

    private boolean check1;
    private boolean check2;

    private double targetAngleDeg = 30;
    private double targetAngleRad = targetAngleDeg * Constants.TAU/360;

    private double posErrorDeg = 1;
    private double posErrorRad = posErrorDeg * Constants.TAU/360;

    private double speedError = 3;

    private double absoluteErrorDeg = 5;
    
    @Override
    public void initialize() {
        check1 = false;
        check2 = false;

        start = true;

        ArmSubsystem.getInstance().setPivotCruiseVelocity(20_000);
        ArmSubsystem.getInstance().setPivotAcceleration(20_000);

        CandleSubsystem.getInstance().set(CandleState.SYSTEM_CHECK);

        ArmSubsystem.getInstance().resetPivotNU();
    }

    @Override
    public void execute() {
        if(!start){
            CandleSubsystem.getInstance().set(CandleState.SYSTEM_BAD);
            SmartDashboard.putString("ENCODER ERROR", e.toString());
        }
        else if(!check1){
            ArmSubsystem.getInstance().pivot(targetAngleRad);
            if(Math.abs(ArmSubsystem.getInstance().getAngle()-targetAngleRad) < posErrorRad 
               && ArmSubsystem.getInstance().getPivotSpeed() < speedError){
                check1 = Math.abs(ArmSubsystem.getInstance().getEncoderDeg()-targetAngleDeg) < absoluteErrorDeg;
                if(check1) CandleSubsystem.getInstance().set(CandleState.PARTIAL_CHECK_1);
                else CandleSubsystem.getInstance().set(CandleState.SYSTEM_BAD);
            }
        }
        else if(!check2){
            ArmSubsystem.getInstance().pivot(-targetAngleRad);
            if(Math.abs(ArmSubsystem.getInstance().getAngle()- (-targetAngleRad)) < posErrorRad
               && ArmSubsystem.getInstance().getPivotSpeed() < speedError){
                check2 = Math.abs(ArmSubsystem.getInstance().getEncoderDeg()- (-targetAngleDeg)) < absoluteErrorDeg;
                if(check2) CandleSubsystem.getInstance().set(CandleState.SYSTEM_GOOD);
                else CandleSubsystem.getInstance().set(CandleState.SYSTEM_BAD);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().setDefaultCruiseVelocity();
        ArmSubsystem.getInstance().setDefaultAcceleration();
        
        ArmSubsystem.getInstance().pivot(0);
    }

    @Override
    public boolean isFinished() {
        return check2;
    }
}
