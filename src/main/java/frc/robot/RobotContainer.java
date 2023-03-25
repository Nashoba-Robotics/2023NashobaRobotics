package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.commands.DriveSpeedCommand;
import frc.robot.commands.intake.DoubleStationIntakeCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeCubeCommand;
import frc.robot.commands.test.GrabberTestCommand;
import frc.robot.commands.test.IntakeTestCommand;
import frc.robot.commands.score.AutoDirectionalPrepHeightCommand;
import frc.robot.commands.score.CubeAutoDirectionalPrepHeightCommand;
import frc.robot.commands.score.PrepHighConeCommand;
import frc.robot.commands.score.PrepHighCubeCommand;
import frc.robot.commands.score.PukeCommand;
import frc.robot.commands.score.ScoreConeCommand;
import frc.robot.commands.score.ScoreCubeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.JoystickSubsytem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.CandleSubsystem.CandleState;

public class RobotContainer {

  public RobotContainer() {
    configureButtonBindings();
    configureTabs();
  }  

  public static CommandJoystick operatorController = new CommandJoystick(2);
  Trigger intakeButton = operatorController.button(2);  //B
  Trigger lowPrepCone = operatorController.button(1); //Y
  Trigger midPrepCone = operatorController.button(3); //A
  Trigger highPrepCone = operatorController.button(4);  //X

  // Trigger doubleStationIntake = operatorController.button(14); //screenshot

  Trigger score = operatorController.button(8); //RT
  //Left cone Right cube

  Trigger cone = operatorController.button(5);  //LB
  Trigger cube = operatorController.button(6);  //RB
  Trigger doubleStation = operatorController.button(7); //LT

  Trigger puke = operatorController.button(10); //+

  Trigger resetGyro = JoystickSubsytem.getInstance().getLeftJoystick().button(1);
  Trigger resetModules = JoystickSubsytem.getInstance().getRightJoystick().button(9);

  Trigger climb90 = JoystickSubsytem.getInstance().getRightJoystick().button(11);
  Trigger climbOpp = JoystickSubsytem.getInstance().getRightJoystick().button(12);
  Trigger setArm0 = JoystickSubsytem.getInstance().getRightJoystick().button(13);

  // Trigger align = JoystickSubsytem.getInstance().getRightJoystick().button(2);

  Trigger eStop = JoystickSubsytem.getInstance().getRightJoystick().button(8);

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

    doubleStation.onTrue(
      new InstantCommand(
        ()-> CandleSubsystem.getInstance().set(CandleState.DOUBLE_STATION),
        CandleSubsystem.getInstance()
        )
    );
    doubleStation.onFalse(new InstantCommand(
      () -> CandleSubsystem.getInstance().set(CandleState.ENABLED),
      CandleSubsystem.getInstance()
      ));

    intakeButton.and(doubleStation).toggleOnTrue(new DoubleStationIntakeCommand());
    
    intakeButton.and(cone).toggleOnTrue(new IntakeCommand(true));

    lowPrepCone.and(cone).onTrue(new AutoDirectionalPrepHeightCommand(TargetLevel.LOW));
    midPrepCone.and(cone).onTrue(new AutoDirectionalPrepHeightCommand(TargetLevel.MID));
    highPrepCone.and(cone).onTrue(new PrepHighConeCommand());

    score.and(cone).toggleOnTrue(new ScoreConeCommand());
    intakeButton.and(cube).toggleOnTrue(new IntakeCubeCommand());

    lowPrepCone.and(cube).onTrue(new CubeAutoDirectionalPrepHeightCommand(TargetLevel.LOW));
    midPrepCone.and(cube).onTrue(new CubeAutoDirectionalPrepHeightCommand(TargetLevel.MID));
    highPrepCone.and(cube).onTrue(new PrepHighCubeCommand());

    score.and(cube).toggleOnTrue(new ScoreCubeCommand());
    puke.toggleOnTrue(new PukeCommand());

    resetGyro.onTrue(new InstantCommand(() -> {
      SwerveDriveSubsystem.getInstance().setGyro(0);
      Pose2d pose = new Pose2d(SwerveDriveSubsystem.getInstance().getPose().getTranslation(), Rotation2d.fromDegrees(0));
      SwerveDriveSubsystem.getInstance().resetOdometry(pose);
    }, SwerveDriveSubsystem.getInstance()));

    resetModules.onTrue(new InstantCommand(() -> 
      SwerveDriveSubsystem.getInstance().resetModulesAbsolute(),
      SwerveDriveSubsystem.getInstance()
    ));

    climb90.onTrue(new InstantCommand(
      () -> {
        ArmSubsystem.getInstance().pivot(Constants.TAU/4);
        ArmSubsystem.getInstance().extendNU(3_000);
      },
      ArmSubsystem.getInstance()
    ));

    climbOpp.onTrue(new InstantCommand(
      () -> {
        ArmSubsystem.getInstance().pivot(-Constants.TAU/4);
        ArmSubsystem.getInstance().extendNU(3_000);
      },
      ArmSubsystem.getInstance()
    ));

    setArm0.onTrue(new InstantCommand(
      () -> {
        ArmSubsystem.getInstance().pivot(0);
        ArmSubsystem.getInstance().extendNU(3_000);
      },
      ArmSubsystem.getInstance()
    ));

    eStop.onTrue(new InstantCommand(
      () -> SwerveDriveSubsystem.getInstance().set(0, 0, 0),
      SwerveDriveSubsystem.getInstance()
    ));
  }

  public void configureTabs() {
    Tabs.Intake.add("Intake Test", new IntakeTestCommand(), 0, 0, 2, 1);
    Tabs.Intake.zeroes.add("Extend", new InstantCommand(
      () -> ArmSubsystem.getInstance().resetPivotNU(),
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
    Tabs.DriveTest.tab.add(new DriveSpeedCommand());

    Tabs.GrabberTest.tab.add(new GrabberTestCommand());
  }
}