package frc.robot.commands.test;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Tabs;
import frc.robot.subsystems.ArmSubsystem;

public class EncoderTestCommand extends CommandBase{
    public EncoderTestCommand(){
        addRequirements(ArmSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ArmSubsystem.getInstance().setPivotBrakeMode(NeutralModeValue.Coast);
    }

    @Override
    public void execute() {
        // Tabs.EncoderTest.displayEncoderValue(ArmSubsystem.getInstance().getEncoderAngle());
        // Tabs.EncoderTest.displayError(ArmSubsystem.getInstance().getLastEncoderError());
        // Tabs.EncoderTest.displayFault(ArmSubsystem.getInstance().getEncoderFault());
        // SmartDashboard.putString("Error", ArmSubsystem.getInstance().getLastEncoderError().name());
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().setPivotBrakeMode(NeutralModeValue.Brake);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
