package frc.robot.commands.auto.intakescore;

import java.io.Console;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoScoreCubeTest extends SequentialCommandGroup{
    public AutoScoreCubeTest(){
        addCommands(
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().zeroWrist();
                ArmSubsystem.getInstance().resetPivotNU();
                SwerveDriveSubsystem.getInstance().setGyro(0);
            }, GrabberSubsystem.getInstance(), ArmSubsystem.getInstance()),
            new InstantCommand(() ->{
                ArmSubsystem.getInstance().pivot(-Constants.Arm.PREP_ANGLE);
                // GrabberSubsystem.getInstance().orient(Constants.Grabber.CUBE_NU);
            }, ArmSubsystem.getInstance(), GrabberSubsystem.getInstance()),
            new WaitCommand(2),
            new AutoScoreCubeCommand()
        );
    }
}
