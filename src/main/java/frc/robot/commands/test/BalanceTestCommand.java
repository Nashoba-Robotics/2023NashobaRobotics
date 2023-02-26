package frc.robot.commands.test;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class BalanceTestCommand extends CommandBase{
    //PigeonIMU pigeon;   //TODO: Get rid of this b/c singleton
    PIDController pigeonController;
    
    public BalanceTestCommand(){
        //pigeon = new PigeonIMU(0);

        pigeonController = new PIDController(0.001, 0, 0);
        pigeonController.setSetpoint(0.05);
        pigeonController.setTolerance(0.01);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        //double roll = SwerveDriveSubsystem.getInstance().getRoll();
        double angle = SwerveDriveSubsystem.getInstance().getBalanceAngle();
        angle = SwerveDriveSubsystem.getInstance().getPitch();
        SmartDashboard.putNumber("Bal Value", pigeonController.calculate(angle));
        SmartDashboard.putNumber("Angle", angle);
        SmartDashboard.putBoolean("Target?", pigeonController.atSetpoint());
        SmartDashboard.putNumber("Roll", SwerveDriveSubsystem.getInstance().getRoll());
        SmartDashboard.putNumber("Pitch", SwerveDriveSubsystem.getInstance().getPitch());

        SwerveDriveSubsystem.getInstance().set(1.5*pigeonController.calculate(angle), 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        SwerveDriveSubsystem.getInstance().set(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return pigeonController.atSetpoint();
    }
}