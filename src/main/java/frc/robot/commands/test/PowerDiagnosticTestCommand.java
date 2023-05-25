package frc.robot.commands.test;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.NewTabs;

public class PowerDiagnosticTestCommand extends CommandBase{
    private PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
    //How do you do the mini power module? Is it just a different CAN ID?

    @Override
    public void execute() {
        for(int i = 0; i < pdh.getNumChannels(); i++){
            NewTabs.putDouble("Power", "Channel " + i, pdh.getCurrent(i));
            
        }

        NewTabs.putBoolean("Power", "Fault 1", pdh.getFaults().Channel0BreakerFault);
    }
}
