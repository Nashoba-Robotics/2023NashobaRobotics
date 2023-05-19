package frc.robot.commands.test;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.NRTalonFX;

public class NRTalonFXTestCommand extends CommandBase{
    NRTalonFX motor = new NRTalonFX(0);

    @Override
    public void execute() {
        motor.set(ControlMode.PercentOutput, 0.1);
    }
}
