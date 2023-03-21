package frc.robot.commands.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.balance.AutoBalanceCommand;
import frc.robot.commands.auto.balance.routine.backToBalance;
import frc.robot.commands.auto.balance.routine.offBalance;
import frc.robot.commands.auto.balance.routine.onToBalance;
import frc.robot.commands.auto.balance.routine.throughBalance;
import frc.robot.commands.auto.balance.routine.waitUntilLevel;
import frc.robot.commands.auto.intakescore.AutoScoreCommand;
import frc.robot.commands.intake.IntakeCubeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class MidToClimb extends SequentialCommandGroup{
    public MidToClimb(){
        addCommands(
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().zeroWrist();
                SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/2);
                SwerveDriveSubsystem.getInstance().resetOdometry(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180)));
                ArmSubsystem.getInstance().pivot(0);
                ArmSubsystem.getInstance().resetPivotNU();
            }, GrabberSubsystem.getInstance(), SwerveDriveSubsystem.getInstance()),
            new InstantCommand(() -> {
                SwerveDriveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
            }, SwerveDriveSubsystem.getInstance()),
            new AutoScoreCommand(), //<-- This makes us tip a bit
            new WaitCommand(1.25), //<-- This makes sure the tip does not mess up the end conditions :)
            new waitUntilLevel(),
            new ParallelCommandGroup(
                new onToBalance(),
                new InstantCommand(
                    () -> ArmSubsystem.getInstance().pivot(-Constants.TAU/4),
                    ArmSubsystem.getInstance()
                )
            ),
            new throughBalance(),
            new offBalance(),
            new WaitCommand(0.5),
            new backToBalance(),
            new AutoBalanceCommand()
        );
    }
}
