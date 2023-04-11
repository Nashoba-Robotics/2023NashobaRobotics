package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerDistributionHubSubsystem extends SubsystemBase{
    private PowerDistribution pdh;

    public PowerDistributionHubSubsystem(){
        pdh = new PowerDistribution(1, ModuleType.kRev);
    }

    public double getTotalVoltage(){
        return pdh.getVoltage();
    }
    public double getTotalCurrent(){
        return pdh.getTotalCurrent();
    }

    public double getCurrent(double channel){
        return pdh.getCurrent(channel);
    }
}
