package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.auto.intakescore.AutoScoreCubeCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class BackSideTest extends SequentialCommandGroup {
    
    public BackSideTest() {
        addCommands(
            new InstantCommand(() -> {
            SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/2);
            }, SwerveDriveSubsystem.getInstance()),
            new AutoScoreCubeCommand()
        );
    }

}
