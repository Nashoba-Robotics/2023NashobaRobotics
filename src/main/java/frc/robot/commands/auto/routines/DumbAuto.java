package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.MoveBackCommand;
import frc.robot.commands.auto.intakescore.AutoScoreCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DumbAuto extends SequentialCommandGroup {
    
    public DumbAuto() {
        addCommands(
            new InstantCommand(() -> SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/2), SwerveDriveSubsystem.getInstance()),
            new WaitCommand(0.5),
            new AutoScoreCommand(),
            new MoveBackCommand()  
        );
    }

}
