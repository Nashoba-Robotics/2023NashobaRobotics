package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CandleSubsystem;

public class LEDTestCommand extends CommandBase{
    
    public LEDTestCommand() {
        addRequirements(CandleSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Red", 0);
        SmartDashboard.putNumber("Green", 0);
        SmartDashboard.putNumber("Blue", 0);
    }

    @Override
    public void execute() {
        double red = SmartDashboard.getNumber("Red", 0);
        double green = SmartDashboard.getNumber("Green", 0);
        double blue = SmartDashboard.getNumber("Blue", 0);
        CandleSubsystem.getInstance().set(new int[]{(int)red, (int)green, (int)blue});
    }

}
