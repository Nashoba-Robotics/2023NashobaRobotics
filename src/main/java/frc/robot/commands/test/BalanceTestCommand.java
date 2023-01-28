package frc.robot.commands.test;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class BalanceTestCommand extends CommandBase{
    //PigeonIMU pigeon;   //TODO: Get rid of this b/c singleton
    PIDController pigeonController;
    
    public BalanceTestCommand(){
        //pigeon = new PigeonIMU(0);

        pigeonController = new PIDController(0.01, 0, 0);
        pigeonController.setSetpoint(1.5);
        pigeonController.setTolerance(0.5);
    }

    @Override
    public void execute() {
        //double roll = SwerveDriveSubsystem.getInstance().getRoll();
        double roll = SwerveDriveSubsystem.getInstance().getRoll();
        SmartDashboard.putNumber("Bal Value", pigeonController.calculate(roll));
        SmartDashboard.putNumber("Roll", roll);
        SmartDashboard.putBoolean("Target?", pigeonController.atSetpoint());

        SwerveDriveSubsystem.getInstance().set(0, 0.3 * pigeonController.calculate(roll), 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}