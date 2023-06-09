package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.KryptonCougarLEDSubsystem;
import frc.robot.subsystems.KryptonCougarLEDSubsystem.LightState;

public class KCLEDTestCommand extends CommandBase{
    @Override
    public void execute() {
        KryptonCougarLEDSubsystem.state = LightState.HAVE_CONE;
    }

    @Override
    public void end(boolean interrupted) {
        KryptonCougarLEDSubsystem.state = LightState.DEFAULT;
    }
}
