package frc.robot.commands.auto.routines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.balance.AutoBalanceCommand;
import frc.robot.commands.auto.balance.routine.offBalance;
import frc.robot.commands.auto.balance.routine.onToBalance;
import frc.robot.commands.auto.balance.routine.throughBalance;
import frc.robot.commands.auto.balance.routine.waitUntilLevel;
import frc.robot.commands.auto.intakescore.AutoScoreCommand;
import frc.robot.commands.auto.move.TranslateToCommand;
import frc.robot.commands.intake.IntakeCubeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class MidToClimbTo1 extends SequentialCommandGroup{

    public MidToClimbTo1(){

        Command translateTo = new TranslateToCommand(new Translation2d(1.55, -0.25), Rotation2d.fromRadians(Constants.TAU/2), new PathConstraints(3.5, 2.5));
        Command translateBack = new TranslateToCommand(new Translation2d(-4, 0.35), Rotation2d.fromRadians(Constants.TAU/2), new PathConstraints(2.75, 2.75));

        addCommands(
            new InstantCommand(() -> {
                GrabberSubsystem.getInstance().zeroWrist();
                SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/2);
                SwerveDriveSubsystem.getInstance().resetOdometryOverrideAngle(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180));
                ArmSubsystem.getInstance().pivot(0);
                ArmSubsystem.getInstance().resetPivotNU();
            }, GrabberSubsystem.getInstance(), SwerveDriveSubsystem.getInstance()),
            new AutoScoreCommand(), //<-- This makes us tip a bit
            new WaitCommand(0.7), //<-- This makes sure the tip does not mess up the end conditions :)
            // new waitUntilLevel(),
            new ParallelCommandGroup(
                new onToBalance(),
                new InstantCommand(
                    () -> ArmSubsystem.getInstance().pivot(-Constants.TAU/4),
                    ArmSubsystem.getInstance()
                )
            ),
            new throughBalance(),
            new offBalance(),
            new WaitCommand(0.1),
            new InstantCommand(() -> ArmSubsystem.getInstance().resetPivotNU(), ArmSubsystem.getInstance()),
            new ParallelCommandGroup(
                translateTo.withTimeout(2),
                new IntakeCubeCommand(true).withTimeout(1.11)
            ),
            translateBack.until(SwerveDriveSubsystem.getInstance()::notLevel),
            new AutoBalanceCommand()
        );
    }
}