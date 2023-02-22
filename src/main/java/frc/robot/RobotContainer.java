package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.commands.ArmAngleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualExtensionCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.auto.intakescore.AutoScoreCommand;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.commands.test.ArmTestCommand;
import frc.robot.commands.test.BalanceTestCommand;
import frc.robot.commands.test.CameraCenterCommand;
import frc.robot.commands.test.CameraTestCommand;
import frc.robot.commands.test.IntakeTestCommand;
import frc.robot.commands.test.LEDTestCommand;
import frc.robot.commands.test.RunMotorCommand;
import frc.robot.commands.score.LowScoreCommand;
import frc.robot.commands.score.PrepHeightCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.test.SwerveDriveTestCommand;
import frc.robot.commands.test.TestAutoCommand;
import frc.robot.commands.test.TestGrabberCommand;
import frc.robot.commands.test.ZeroPivotCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.JoystickSubsytem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.HashMap;
import java.util.Map;

public class RobotContainer {
  public Map<String, Command> eventMap = new HashMap<>();

  public RobotContainer() {
    configureButtonBindings();
    SmartDashboard.putData(new SwerveDriveCommand());
    SmartDashboard.putData(new TestAutoCommand());
    // SmartDashboard.putData(new SwerveDriveTestCommand());
    //SmartDashboard.putData(new RunMotorCommand());
    // SmartDashboard.putData(new FollowPathCommand(PathPlanner.loadPath("testPath", new PathConstraints(4, 2))));
    // SmartDashboard.putData(new CameraTestCommand());
    // SmartDashboard.putData(new BalanceTestCommand());
    SmartDashboard.putData(new ZeroPivotCommand());
    // SmartDashboard.putData(new ArmTestCommand());
    SmartDashboard.putData("ZeroArmSensor", new InstantCommand(() -> ArmSubsystem.getInstance().zeroArmSensor(), ArmSubsystem.getInstance()));
    SmartDashboard.putData("ZeroWristSensor", new InstantCommand(() -> GrabberSubsystem.getInstance().zeroWrist(), GrabberSubsystem.getInstance()));

    //SmartDashboard.putData(new TestGrabberCommand());
    SmartDashboard.putData(new ManualExtensionCommand());
    SmartDashboard.putData(new IntakeTestCommand());
    // SmartDashboard.putData(new IntakeCommand());
    // SmartDashboard.putData(new CameraCenterCommand());
    // SmartDashboard.putData(new BalanceTestCommand());

    // SmartDashboard.putData("Prep High",new PrepHeightCommand(TargetLevel.HIGH));
    // SmartDashboard.putData("Prep Mid", new PrepHeightCommand(TargetLevel.MID));
    SmartDashboard.putData("Zero Arm command", new ArmAngleCommand(0));
    // SmartDashboard.putData("Score Command", new ScoreCommand());

    SmartDashboard.putData("Reset Gyro", new InstantCommand(() -> SwerveDriveSubsystem.getInstance().setGyro(0), SwerveDriveSubsystem.getInstance()));
    SmartDashboard.putData("Reset Odometery", new InstantCommand(() -> SwerveDriveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0))), SwerveDriveSubsystem.getInstance()));

    SmartDashboard.putData("AutoScore", new AutoScoreCommand());

    eventMap.put("Intake Start", new IntakeCommand(true));
    eventMap.put("Stop Intake", new InstantCommand(
        () -> {
            ArmSubsystem.getInstance().pivot(0);
            GrabberSubsystem.getInstance().set(-0.1);
        },
        ArmSubsystem.getInstance(),
        GrabberSubsystem.getInstance()
    ));

    SmartDashboard.putData(new LEDTestCommand());
    Tabs.Intake.add("Intake Test", new IntakeTestCommand(), 0, 0, 2, 1);
    Tabs.Intake.zeroes.add(new InstantCommand(
      () -> ArmSubsystem.getInstance().zeroArm(),
      ArmSubsystem.getInstance()
    ));
    Tabs.Intake.zeroes.add(new InstantCommand(
      () -> ArmSubsystem.getInstance().zeroPivot(),
      ArmSubsystem.getInstance()
    ));
    Tabs.Intake.zeroes.add(new InstantCommand(
      () -> GrabberSubsystem.getInstance().zeroWrist(),
      GrabberSubsystem.getInstance()
    ));
  }

  public static CommandJoystick operatorController = new CommandJoystick(2);
  Trigger intakeButton = operatorController.button(2);  //B
  Trigger lowPrepCone = operatorController.button(1); //Y
  Trigger midPrepCone = operatorController.button(3); //A
  Trigger highPrepCone = operatorController.button(4);  //X

  Trigger scoreCone = operatorController.button(8); //RT
  Trigger lowScore = operatorController.button(7);  //LT

  Trigger resetGyro = JoystickSubsytem.getInstance().getLeftJoystick().button(1);
  Trigger resetModules = JoystickSubsytem.getInstance().getRightJoystick().button(1);

  public void configureButtonBindings(){
    intakeButton.toggleOnTrue(new IntakeCommand(true));

    lowPrepCone.onTrue(new PrepHeightCommand(TargetLevel.LOW));
    midPrepCone.onTrue(new PrepHeightCommand(TargetLevel.MID));
    highPrepCone.onTrue(new PrepHeightCommand(TargetLevel.HIGH));

    scoreCone.toggleOnTrue(new ScoreCommand());
    lowScore.toggleOnTrue(new LowScoreCommand());

    resetGyro.onTrue(new InstantCommand(() -> {
      SwerveDriveSubsystem.getInstance().setGyro(0);
      SwerveDriveSubsystem.getInstance().resetOdometry(SwerveDriveSubsystem.getInstance().getPose());
    }, SwerveDriveSubsystem.getInstance()));

    resetModules.onTrue(new InstantCommand(() -> 
      SwerveDriveSubsystem.getInstance().resetModulesAbsolute(),
      SwerveDriveSubsystem.getInstance()
    ));
  }
}
