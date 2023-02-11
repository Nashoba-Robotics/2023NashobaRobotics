package frc.robot.commands.test;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunMotorCommand extends CommandBase {
    
    CANSparkMax motor;

    // public RunMotorCommand() {
    //     //motor = new CANSparkMax(13, MotorType.kBrushless);
        
    //     motor.setIdleMode(IdleMode.kBrake);
    // }

    // @Override
    // public void initialize() {
    //     SmartDashboard.putNumber("Motor Speed", 0);
    // }

    // @Override
    // public void execute() {
    //     double speed = SmartDashboard.getNumber("Motor Speed", 0);
    //     motor.set(speed > 0.5 ? 0.5 : speed);
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     motor.set(0);
    // }

    // @Override
    // public boolean isFinished() {
    //     return false;
    // }

}
