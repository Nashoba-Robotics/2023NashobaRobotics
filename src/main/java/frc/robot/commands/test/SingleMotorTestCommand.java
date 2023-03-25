package frc.robot.commands.test;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SingleMotorTestCommand extends CommandBase {
    
    CANSparkMax motor;

    public SingleMotorTestCommand() {
        motor = new CANSparkMax(12, MotorType.kBrushless);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("MotorSpeed", 0);
    }

    @Override
    public void execute() {
        motor.set(SmartDashboard.getNumber("MotorSpeed", 0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
