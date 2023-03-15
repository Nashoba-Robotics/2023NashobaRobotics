package frc.robot.commands.auto.intakescore;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoScoreTest extends SequentialCommandGroup{
    public AutoScoreTest(){
        addCommands(
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().zeroWrist();
                ArmSubsystem.getInstance().resetPivotNU();
                SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/2);
            }, GrabberSubsystem.getInstance(), ArmSubsystem.getInstance()),
            new AutoScoreCommand()
        );
    }
}
