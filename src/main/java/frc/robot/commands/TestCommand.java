package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TestCommand extends CommandBase {
    private TalonFX motor = new TalonFX(0);
    Timer timer = new Timer();
    boolean end = false;
    public TestCommand() {
        //addRequirements(SwerveDriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Moved", false);
        motor.setSelectedSensorPosition(0);
        motor.set(ControlMode.MotionMagic, 10_000);
        timer.start();
    }

    @Override
    public void execute() {
        if(timer.get() > 2){
            
            SmartDashboard.putBoolean("Moved", true);
            motor.setSelectedSensorPosition(0);
            end = true;
        }
        SmartDashboard.putNumber("Motor Pos", motor.getSelectedSensorPosition());
    }

    @Override
    public void end(boolean interrupted) {
       // SwerveDriveSubsystem.getInstance().setDirectly(0, 0);
    }

    @Override
    public boolean isFinished() {
        return end;
    }
    
}
