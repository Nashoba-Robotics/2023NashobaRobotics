package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.auto.MoveBackCommand;
import frc.robot.commands.auto.intakescore.AutoScoreCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DumbAuto extends SequentialCommandGroup {
    
    public DumbAuto() {
        addCommands(
            new InstantCommand(() -> SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/4), SwerveDriveSubsystem.getInstance()),
            new AutoScoreCommand(),
            new MoveBackCommand()  
        );
    }

}
