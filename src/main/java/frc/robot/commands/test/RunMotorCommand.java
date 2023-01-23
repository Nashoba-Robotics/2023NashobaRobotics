package frc.robot.commands.test;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunMotorCommand extends CommandBase {
    
    TalonFX motor;

    public RunMotorCommand() {
        motor = new TalonFX(0);
    }

    @Override
    public void execute() {
        motor.set(ControlMode.PercentOutput, 0.2);
    }

    @Override
    public void end(boolean interrupted) {
        motor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
