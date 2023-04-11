package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Tabs;
import frc.robot.subsystems.ArmSubsystem;

public class TestPivotCommand extends CommandBase {
    
    private double P;
    private double I;
    private double D;

    public TestPivotCommand() {
        addRequirements(ArmSubsystem.getInstance());
        P = 0;
        I = 0;
        D = 0;
    }

    @Override
    public void execute() {
        ArmSubsystem.getInstance().setAnglePID(Tabs.PivotTest.getSetAngle());
        if(P != Tabs.PivotTest.getP()) {
            P = Tabs.PivotTest.getP();
            ArmSubsystem.getInstance().setP(P);
        }
        if(I != Tabs.PivotTest.getI()) {
            I = Tabs.PivotTest.getI();
            ArmSubsystem.getInstance().setI(I);
        }
        if(D != Tabs.PivotTest.getD()) {
            D = Tabs.PivotTest.getD();
            ArmSubsystem.getInstance().setD(D);
        }

        Tabs.PivotTest.displayActualAngle(ArmSubsystem.getInstance().getEncoderAngle());
        Tabs.PivotTest.displayArmSpeed(ArmSubsystem.getInstance().getPivotSpeed());
    }

}
