package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.move.TranslateToCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;


public class CameraCenterCommand extends SequentialCommandGroup{

    public CameraCenterCommand(){
        addCommands(
            new InstantCommand(() -> SwerveDriveSubsystem.getInstance().resetOdometry(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0))), SwerveDriveSubsystem.getInstance())
            // new TranslateToCommand(new Translation2d(0, 1), Rotation2d.fromRadians(0))
        );
    }

}
