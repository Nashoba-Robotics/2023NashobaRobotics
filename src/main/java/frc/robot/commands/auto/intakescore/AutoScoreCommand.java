package frc.robot.commands.auto.intakescore;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Grabber;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoScoreCommand extends SequentialCommandGroup {
    
    public AutoScoreCommand() {
        addCommands(
            new InstantCommand(() -> {
                ArmSubsystem.getInstance().pivot(Constants.Arm.HIGH_ANGLE);
            }, ArmSubsystem.getInstance()),
            new WaitCommand(2),
            new InstantCommand(() -> {
                ArmSubsystem.getInstance().extendNU(Constants.Arm.HIGH_EXTEND_NU);
                GrabberSubsystem.getInstance().orientPos(Constants.Grabber.HIGH_ANGLE);
            }, ArmSubsystem.getInstance(), GrabberSubsystem.getInstance()),
            new WaitCommand(2),
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().score();
            }, GrabberSubsystem.getInstance()),
            new WaitCommand(2),
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().stop();
                ArmSubsystem.getInstance().pivot(0);
            }, ArmSubsystem.getInstance(), GrabberSubsystem.getInstance())
        );
    }

}
