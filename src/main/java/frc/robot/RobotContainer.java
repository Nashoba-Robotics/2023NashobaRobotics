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
import frc.robot.commands.IntakeCubeCommand;
import frc.robot.commands.ManualExtensionCommand;
import frc.robot.commands.SwerveDriveCommand;
// import frc.robot.commands.auto.balance.AutoBalanceCommand;
import frc.robot.commands.auto.balance.BalanceCommand;
import frc.robot.commands.auto.balance.BalanceCommand.Balance;
import frc.robot.commands.auto.intakescore.AutoScoreCommand;
import frc.robot.commands.auto.lib.FollowPathCommand;
import frc.robot.commands.test.CameraCenterCommand;
import frc.robot.commands.test.CameraTestCommand;
import frc.robot.commands.test.DriveToTestCommand;
import frc.robot.commands.test.IntakeTestCommand;
import frc.robot.commands.score.AutoDirectionalPrepHeightCommand;
import frc.robot.commands.score.CubeAutoDirectionalPrepHeightCommand;
import frc.robot.commands.score.LowScoreCommand;
import frc.robot.commands.score.PrepHeightCommand;
import frc.robot.commands.score.ScoreConeCommand;
import frc.robot.commands.score.ScoreCubeCommand;
import frc.robot.commands.test.TestAutoCommand;
import frc.robot.commands.test.ZeroPivotCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.JoystickSubsytem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.CandleSubsystem.CandleState;

import java.util.HashMap;
import java.util.Map;

public class RobotContainer {
  public Map<String, Command> eventMap = new HashMap<>();

  public RobotContainer() {
    configureButtonBindings();

    // SmartDashboard.putData(new IntakeCubeCommand());
    // SmartDashboard.putData(new CubeAutoDirectionalPrepHeightCommand(TargetLevel.HIGH));
    // SmartDashboard.putData(new ScoreCubeCommand());
    // SmartDashboard.putData(new SwerveDriveCommand());
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

    // SmartDashboard.putData(new CameraCenterCommand());

    // SmartDashboard.putData("Prep High",new PrepHeightCommand(TargetLevel.HIGH));
    // SmartDashboard.putData("Prep Mid", new PrepHeightCommand(TargetLevel.MID));
    SmartDashboard.putData("Zero Arm command", new ArmAngleCommand(0));
    // SmartDashboard.putData("Score Command", new ScoreCommand());

    // SmartDashboard.putData("Reset Gyro", new InstantCommand(() -> SwerveDriveSubsystem.getInstance().setGyro(0), SwerveDriveSubsystem.getInstance()));
    // SmartDashboard.putData("Reset Odometery", new InstantCommand(() -> SwerveDriveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0))), SwerveDriveSubsystem.getInstance()));

    // SmartDashboard.putData("AutoScore", new AutoScoreCommand());

    SmartDashboard.putData("Brake", new InstantCommand(() -> SwerveDriveSubsystem.getInstance().brake()));
    
    SmartDashboard.putData("DriveTo", new DriveToTestCommand());

    // SmartDashboard.putData("Balance", new AutoBalanceCommand());

    SmartDashboard.putData(new AutoScoreCommand());

    eventMap.put("Intake Start", new IntakeCommand(true));
    eventMap.put("Stop Intake", new InstantCommand(
        () -> {
            ArmSubsystem.getInstance().pivot(0);
            GrabberSubsystem.getInstance().set(-0.1);
        },
        ArmSubsystem.getInstance(),
        GrabberSubsystem.getInstance()
    ));

    Tabs.Intake.add("Intake Test", new IntakeTestCommand(), 0, 0, 2, 1);
    Tabs.Intake.zeroes.add("Extend", new InstantCommand(
      () -> ArmSubsystem.getInstance().zeroArmSensor(),
      ArmSubsystem.getInstance()
    ));
    Tabs.Intake.zeroes.add("Pivot",new InstantCommand(
      () -> ArmSubsystem.getInstance().zeroPivotSensor(),
      ArmSubsystem.getInstance()
    ));
    Tabs.Intake.zeroes.add("Wrist", new InstantCommand(
      () -> GrabberSubsystem.getInstance().zeroWrist(),
      GrabberSubsystem.getInstance()
    ));
  }

  public static CommandJoystick operatorController = new CommandJoystick(2);
  Trigger intakeButton = operatorController.button(2);  //B
  Trigger lowPrepCone = operatorController.button(1); //Y
  Trigger midPrepCone = operatorController.button(3); //A
  Trigger highPrepCone = operatorController.button(4);  //X

  Trigger score = operatorController.button(8); //RT
  Trigger lowScore = operatorController.button(7);  //LT

  //Left cone Right cube

  Trigger cone = operatorController.button(5);  //LB
  Trigger cube = operatorController.button(6);  //RB

  Trigger resetGyro = JoystickSubsytem.getInstance().getLeftJoystick().button(1);
  Trigger resetModules = JoystickSubsytem.getInstance().getRightJoystick().button(1);

  public void configureButtonBindings(){
    cone.onTrue(new InstantCommand(
      () -> CandleSubsystem.getInstance().set(CandleState.WANT_CONE),
      CandleSubsystem.getInstance()
      ));
    cone.onFalse(new InstantCommand(
      () -> CandleSubsystem.getInstance().set(CandleState.ENABLED),
      CandleSubsystem.getInstance()
    ));

    cube.onTrue(new InstantCommand(
      () -> CandleSubsystem.getInstance().set(CandleState.WANT_CUBE),
      CandleSubsystem.getInstance()
    ));
    cube.onFalse(new InstantCommand(
      () -> CandleSubsystem.getInstance().set(CandleState.ENABLED),
      CandleSubsystem.getInstance()
    ));
    
    intakeButton.and(cone).toggleOnTrue(new IntakeCommand(true));

    lowPrepCone.and(cone).onTrue(new AutoDirectionalPrepHeightCommand(TargetLevel.LOW));
    midPrepCone.and(cone).onTrue(new AutoDirectionalPrepHeightCommand(TargetLevel.MID));
    highPrepCone.and(cone).onTrue(new AutoDirectionalPrepHeightCommand(TargetLevel.HIGH));

    score.and(cone).toggleOnTrue(new ScoreConeCommand());
    lowScore.and(cone).toggleOnTrue(new LowScoreCommand());

    intakeButton.and(cube).toggleOnTrue(new IntakeCubeCommand());

    lowPrepCone.and(cube).onTrue(new CubeAutoDirectionalPrepHeightCommand(TargetLevel.LOW));
    midPrepCone.and(cube).onTrue(new CubeAutoDirectionalPrepHeightCommand(TargetLevel.MID));
    highPrepCone.and(cube).onTrue(new CubeAutoDirectionalPrepHeightCommand(TargetLevel.HIGH));

    score.and(cube).toggleOnTrue(new ScoreCubeCommand());
    lowScore.and(cube).toggleOnTrue(new LowScoreCommand());



    resetGyro.onTrue(new InstantCommand(() -> {
      SwerveDriveSubsystem.getInstance().setGyro(0);
      Pose2d pose = new Pose2d(SwerveDriveSubsystem.getInstance().getPose().getTranslation(), Rotation2d.fromDegrees(0));
      SwerveDriveSubsystem.getInstance().resetOdometry(pose);
    }, SwerveDriveSubsystem.getInstance()));

    resetModules.onTrue(new InstantCommand(() -> 
      SwerveDriveSubsystem.getInstance().resetModulesAbsolute(),
      SwerveDriveSubsystem.getInstance()
    ));
  }
}
