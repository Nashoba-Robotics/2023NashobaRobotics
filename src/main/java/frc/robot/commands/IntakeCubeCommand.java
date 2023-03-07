package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class IntakeCubeCommand extends CommandBase {
    boolean joystick02;
    double lastPos2;

    double setPos2;
    boolean atSetPoint2;

    public IntakeCubeCommand(){
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        GrabberSubsystem.getInstance().setCurrentLimit(30);

        ArmSubsystem.getInstance().setPivotCruiseVelocity(40_000);
        ArmSubsystem.getInstance().setPivotAcceleration(40_000);

        // Extend is TEMP to test at the same distance
        ArmSubsystem.getInstance().extendNU(3_000);
        ArmSubsystem.getInstance().pivot(Constants.Arm.Cube.INTAKE_ANGLE);
        // setPos2 = -108;
        atSetPoint2 = false;
        joystick02 = false;
        GrabberSubsystem.getInstance().orientPos(-9);
        // lastPos2 = ArmSubsystem.getInstance().getAngle();
    }

    @Override
    public void execute() {
        GrabberSubsystem.getInstance().set(0.4, -0.4);
        SmartDashboard.putNumber("Grabber Current", GrabberSubsystem.getInstance().getCurrent());

        // if(Math.abs(ArmSubsystem.getInstance().getAngle() - setPos2) < 0.5 * Constants.TAU/360){
        //     atSetPoint2 = true;
        //     // lastPos2 = ArmSubsystem.getInstance().getAngle();
        // } 

        // if(atSetPoint2) {
        //     double pivotX = RobotContainer.operatorController.getX();
        //     pivotX = Math.abs(pivotX) < 0.1 ? 0 : (pivotX-0.1)/0.9;
        //     if(pivotX == 0){ // If there isn't any input, maintain the position
        //         // ArmSubsystem.getInstance().pivot(ArmSubsystem.getInstance().getAngle());
        //         if(!joystick02){
        //             joystick02 = true;
        //             lastPos2 = ArmSubsystem.getInstance().getAngle();
        //         }
        //         ArmSubsystem.getInstance().pivot(lastPos2);
        //     }
        //     else{
        //         ArmSubsystem.getInstance().setPivot(pivotX*0.13);
        //         joystick02 = false;
        //     }
        // }
    }

    @Override
    public void end(boolean interrupted) {
        GrabberSubsystem.getInstance().setCurrentLimit(10);
        ArmSubsystem.getInstance().pivot(0);
        // GrabberSubsystem.getInstance().orient(0);
        GrabberSubsystem.getInstance().set(0);   //Make the grabber hold it
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
