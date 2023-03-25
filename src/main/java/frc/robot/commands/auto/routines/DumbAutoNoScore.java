package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.move.MoveBackCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DumbAutoNoScore extends SequentialCommandGroup {
    
    public DumbAutoNoScore() {
        addCommands(
            new InstantCommand(() -> SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/2), SwerveDriveSubsystem.getInstance()),
            new WaitCommand(0.5),
            new MoveBackCommand()  
        );
    }

}
