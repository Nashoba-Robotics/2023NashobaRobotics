package frc.robot.commands.auto.systemcheck;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.IntakeCommand;
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
            new IntakeCommand(true).until(GrabberSubsystem.getInstance()::hasCone),
            new InstantCommand(
                () -> {
                    ArmSubsystem.getInstance().pivot(0);
                }
            ).until(ArmSubsystem.getInstance()::armAtZero)
        );
    }
}
