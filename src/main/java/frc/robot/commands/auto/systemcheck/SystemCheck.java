package frc.robot.commands.auto.systemcheck;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.commands.score.ScoreConeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.CandleSubsystem.CandleState;

public class SystemCheck extends SequentialCommandGroup{
    public SystemCheck(){
        addCommands(
            new InstantCommand(
                () -> {
                    SwerveDriveSubsystem.getInstance().setGyro(0);
                    GrabberSubsystem.getInstance().zeroWrist();
                    ArmSubsystem.getInstance().zeroExtend();
                },
                ArmSubsystem.getInstance(), GrabberSubsystem.getInstance(), SwerveDriveSubsystem.getInstance()
            ),
            new EncoderCheck(),
            new WaitCommand(0.5),
            new InstantCommand(
                () -> CandleSubsystem.getInstance().set(CandleState.SYSTEM_CHECK),
                CandleSubsystem.getInstance()
            ),
            new IntakeConeCheck(),
            new WaitCommand(0.5),
            new InstantCommand(
                () -> CandleSubsystem.getInstance().set(CandleState.SYSTEM_CHECK),
                CandleSubsystem.getInstance()
            )
            // new ConePrepCheck(TargetLevel.HIGH),
            // new ScoreConeCommand(),
            // new WaitCommand(0.75),
            // new InstantCommand(
            //     () -> CandleSubsystem.getInstance().set(CandleState.SYSTEM_CHECK),
            //     CandleSubsystem.getInstance()
            // ),
            // new ConePrepCheck(TargetLevel.MID),
            // new ScoreConeCommand(),
            // new WaitCommand(0.75),
            // new InstantCommand(
            //     () -> CandleSubsystem.getInstance().set(CandleState.SYSTEM_CHECK),
            //     CandleSubsystem.getInstance()
            // ),
            // new ConePrepCheck(TargetLevel.LOW),
            // new ScoreConeCommand(),
            // new InstantCommand(
            //     () -> CandleSubsystem.getInstance().set(CandleState.ENABLED),
            //     CandleSubsystem.getInstance()
            // )
        );
    }
}
