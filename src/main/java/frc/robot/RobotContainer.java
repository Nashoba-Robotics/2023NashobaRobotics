package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualExtensionCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.test.ArmTestCommand;
import frc.robot.commands.test.BalanceTestCommand;
import frc.robot.commands.test.CameraTestCommand;
import frc.robot.commands.test.IntakeTestCommand;
import frc.robot.commands.test.RunMotorCommand;
import frc.robot.commands.auto.FollowPathCommand;
import frc.robot.commands.score.PrepHeightCommand;
import frc.robot.commands.test.SwerveDriveTestCommand;
import frc.robot.commands.test.TestGrabberCommand;
import frc.robot.commands.test.ZeroPivotCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class RobotContainer {

  public RobotContainer() {
    SmartDashboard.putData(new SwerveDriveCommand());
    // SmartDashboard.putData(new SwerveDriveTestCommand());
    //SmartDashboard.putData(new RunMotorCommand());
    // SmartDashboard.putData(new FollowPathCommand(PathPlanner.loadPath("testPath", new PathConstraints(4, 2))));
    // SmartDashboard.putData(new CameraTestCommand());
    // SmartDashboard.putData(new BalanceTestCommand());
    SmartDashboard.putData(new ZeroPivotCommand());
    SmartDashboard.putData(new ArmTestCommand());
    SmartDashboard.putData(new InstantCommand(() -> ArmSubsystem.getInstance().zeroArm(), ArmSubsystem.getInstance()));
    SmartDashboard.putData(new InstantCommand(() -> GrabberSubsystem.getInstance().zeroWrist(), GrabberSubsystem.getInstance()));

    //SmartDashboard.putData(new TestGrabberCommand());
    SmartDashboard.putData(new ManualExtensionCommand());
    SmartDashboard.putData(new IntakeTestCommand());
    SmartDashboard.putData(new IntakeCommand());

    SmartDashboard.putData("Prep High",new PrepHeightCommand(TargetLevel.HIGH));
    SmartDashboard.putData("Prep Mid",new PrepHeightCommand(TargetLevel.MID));

  }
}
