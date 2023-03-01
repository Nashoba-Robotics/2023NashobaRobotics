package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.MoveBackCommand;
import frc.robot.commands.auto.intakescore.AutoScoreCommand;

public class DumbAuto extends SequentialCommandGroup {
    
    public DumbAuto() {
        addCommands(
            new AutoScoreCommand(),
            new MoveBackCommand()  
        );
    }

}
