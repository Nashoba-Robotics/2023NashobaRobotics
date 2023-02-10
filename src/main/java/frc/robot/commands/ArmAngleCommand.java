package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmAngleCommand extends CommandBase {
    private double angle;

    public ArmAngleCommand(double angle) {
        addRequirements(ArmSubsystem.getInstance());
        this.angle = angle;
    }

    @Override
    public void initialize() {
        ArmSubsystem.getInstance().pivot(angle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
