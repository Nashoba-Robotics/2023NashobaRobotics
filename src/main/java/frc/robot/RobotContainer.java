package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.commands.ArmAngleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualExtensionCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.test.ArmTestCommand;
import frc.robot.commands.test.BalanceTestCommand;
import frc.robot.commands.test.CameraCenterCommand;
import frc.robot.commands.test.CameraTestCommand;
import frc.robot.commands.test.IntakeTestCommand;
import frc.robot.commands.test.RunMotorCommand;
import frc.robot.commands.auto.FollowPathCommand;
import frc.robot.commands.score.PrepHeightCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.test.SwerveDriveTestCommand;
import frc.robot.commands.test.TestGrabberCommand;
import frc.robot.commands.test.ZeroPivotCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class RobotContainer {

  public RobotContainer() {
    configureButtonBindings();
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
    // SmartDashboard.putData(new CameraCenterCommand());

    // SmartDashboard.putData(new BalanceTestCommand());

    SmartDashboard.putData("Prep High",new PrepHeightCommand(TargetLevel.HIGH));
    SmartDashboard.putData("Prep Mid", new PrepHeightCommand(TargetLevel.MID));
    SmartDashboard.putData("Zero Arm command", new ArmAngleCommand(0));
    SmartDashboard.putData("Score Command", new ScoreCommand());

  }

  public static CommandJoystick operatorController = new CommandJoystick(2);
  Trigger intakeButton = operatorController.button(2);  //B
  Trigger lowPrepCone = operatorController.button(1);
  Trigger midPrepCone = operatorController.button(3); //A
  Trigger highPrepCone = operatorController.button(4);  //X

  Trigger scoreCone = operatorController.button(8); //RT
  Trigger pivotMan = operatorController.button(7);  //LT

  public void configureButtonBindings(){
    intakeButton.toggleOnTrue(new IntakeCommand());

    //midPrepCone.onTrue(new PrepHeightCommand(TargetLevel.MID));
    highPrepCone.onTrue(new PrepHeightCommand(TargetLevel.HIGH));

    scoreCone.toggleOnTrue(new ScoreCommand());
  }
}
