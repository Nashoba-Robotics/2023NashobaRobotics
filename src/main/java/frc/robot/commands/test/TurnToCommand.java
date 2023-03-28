package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

//Essentially, robot just turns towards target and extends out (Tunk and Dunk)
public class TurnToCommand extends CommandBase{
    private double ty;
    private double tx;

    private boolean turned; //Has the robot turned towards the target?

    public TurnToCommand(){
        addRequirements(SwerveDriveSubsystem.getInstance(), 
        ArmSubsystem.getInstance(), 
        GrabberSubsystem.getInstance()
        );
    }

    @Override
    public void initialize() {
        tx = LimelightSubsystem.getInstance().getTX();
        ty = LimelightSubsystem.getInstance().getTY();
    }

    @Override
    public void execute() {
        //Turn robot based on tx


        //Extends based on ty
        if(turned){

        }        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
