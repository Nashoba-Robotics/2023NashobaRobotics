package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class IntakeCommand extends CommandBase {
    boolean joystick02 = false;
    double lastPos2;

    double setPos2;
    boolean atSetPoint2;

    int multiplier;

    // public IntakeCommand(double armAngle, double wristAngle) {
    //     addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());

    //     this.armAngle = armAngle;
    // }

    public IntakeCommand(boolean intakeFront){
        multiplier = intakeFront ? 1 : -1;
        addRequirements(ArmSubsystem.getInstance(), GrabberSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ArmSubsystem.getInstance().setCruiseVelocity(400_000);
        ArmSubsystem.getInstance().setAceleration(60_000);

        // Extend is TEMP to test at the same distance
        ArmSubsystem.getInstance().extendNU(0);
        ArmSubsystem.getInstance().pivot(Constants.Arm.INTAKE_ANGLE * multiplier);
        setPos2 = Constants.Arm.INTAKE_ANGLE * multiplier;
        atSetPoint2 = false;
        GrabberSubsystem.getInstance().orientPos(Constants.Grabber.INTAKE_ANGLE * multiplier);
        lastPos2 = ArmSubsystem.getInstance().getAngle();
    }

    @Override
    public void execute() {
        GrabberSubsystem.getInstance().intake();
        SmartDashboard.putNumber("Grabber Current", GrabberSubsystem.getInstance().getCurrent());

        if(Math.abs(ArmSubsystem.getInstance().getAngle() - setPos2) < Constants.TAU / 40){
            atSetPoint2 = true;
            lastPos2 = ArmSubsystem.getInstance().getAngle();
        } 

        if(atSetPoint2) {
            double pivotX = RobotContainer.operatorController.getX();
            pivotX = Math.abs(pivotX) < 0.1 ? 0 : (pivotX-0.1)/0.9;
            if(pivotX == 0){ // If there isn't any input, maintain the position
                // ArmSubsystem.getInstance().pivot(ArmSubsystem.getInstance().getAngle());
                if(!joystick02){
                    joystick02 = true;
                    lastPos2 = ArmSubsystem.getInstance().getAngle();
                }
                ArmSubsystem.getInstance().pivot(lastPos2);
            }
            else{
                ArmSubsystem.getInstance().setPivot(pivotX*0.13);
                joystick02 = false;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().pivot(0);
        GrabberSubsystem.getInstance().orient(0);
        GrabberSubsystem.getInstance().set(-0.1);   //Make the grabber hold it
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
