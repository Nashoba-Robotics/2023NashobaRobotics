package frc.robot;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.commands.test.ArmSpeedTestCommand;
import frc.robot.commands.test.CameraCenterCommand;
import frc.robot.commands.test.EncoderTestCommand;
import frc.robot.commands.test.IntakeTestCommand;
import frc.robot.commands.test.KCLEDTestCommand;
import frc.robot.commands.test.RunArmCommand;
import frc.robot.commands.SetPivotOffsetCommand;
import frc.robot.commands.intake.SingleStationIntakeCommand;
import frc.robot.commands.intake.IntakeConeCommand;
import frc.robot.commands.intake.IntakeCubeCommand;
import frc.robot.commands.score.AutoDirectionalPrepHeightCommand;
import frc.robot.commands.score.CubeAutoDirectionalPrepHeightCommand;
import frc.robot.commands.score.PukeCommand;
import frc.robot.commands.score.ScoreConeCommand;
import frc.robot.commands.score.ScoreCubeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.KryptonCougarLEDSubsystem;
import frc.robot.subsystems.SmartLEDSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.CandleSubsystem.CandleState;
import frc.robot.subsystems.SmartLEDSubsystem.LightState;

public class RobotContainer {

  public RobotContainer() {
    configureButtonBindings();
    configureTabs();

    // SmartDashboard.putData(new CameraCenterCommand());
    SmartDashboard.putData(new InstantCommand(
      () -> SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/2)
    ));
    SmartDashboard.putData(new KCLEDTestCommand());
    // SmartDashboard.putData(new RunArmCommand());
    // SmartDashboard.putData(new EncoderTestCommand());

    NewTabs.putCommand("Arm Speed Test", new ArmSpeedTestCommand());
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

  Trigger resetGyro = JoystickSubsystem.getInstance().getLeftJoystick().button(1);
  Trigger resetModules = JoystickSubsystem.getInstance().getRightJoystick().button(9);

  Trigger climb90 = JoystickSubsystem.getInstance().getRightJoystick().button(11);
  Trigger climbOpp = JoystickSubsystem.getInstance().getRightJoystick().button(12);
  Trigger setArm0 = JoystickSubsystem.getInstance().getRightJoystick().button(13);

  Trigger align = JoystickSubsystem.getInstance().getRightJoystick().button(2);

  Trigger eStop = JoystickSubsystem.getInstance().getRightJoystick().button(8);

  Trigger squareUpAndSetArm = JoystickSubsystem.getInstance().getRightJoystick().button(1);

  Trigger resetPivotNU = JoystickSubsystem.getInstance().getRightJoystick().button(10);

  public void configureButtonBindings(){

    // align.onTrue(new DriveToTestCommand());

    squareUpAndSetArm.onTrue(new InstantCommand(() -> {
      int multiplier = SwerveDriveSubsystem.getInstance().getGyroAngle() < Constants.TAU/4 &&
        SwerveDriveSubsystem.getInstance().getGyroAngle() > -Constants.TAU/4 ? -1 : 1;
        ArmSubsystem.getInstance().pivot(multiplier*Constants.Arm.PREP_ANGLE);
        ArmSubsystem.getInstance().extendNU(Constants.Arm.EXTEND_REST_NU);
    },
      ArmSubsystem.getInstance()));

    cone.onTrue(new InstantCommand(
      () -> SmartLEDSubsystem.state = LightState.WANT_CONE
      ));
    cone.onFalse(new InstantCommand(
      () -> SmartLEDSubsystem.state = LightState.DEFAULT
    ));

    cube.onTrue(new InstantCommand(
      () -> SmartLEDSubsystem.state = LightState.WANT_CUBE
    ));
    cube.onFalse(new InstantCommand(
      () -> SmartLEDSubsystem.state = LightState.DEFAULT
    ));

    // doubleStation.onTrue(
    //   new InstantCommand(
    //     ()-> CandleSubsystem.getInstance().set(CandleState.DOUBLE_STATION),
    //     CandleSubsystem.getInstance()
    //     )
    // );
    // doubleStation.onFalse(new InstantCommand(
    //   () -> CandleSubsystem.getInstance().set(CandleState.ENABLED),
    //   CandleSubsystem.getInstance()
    //   ));

    intakeButton.and(doubleStation).toggleOnTrue(new SingleStationIntakeCommand());
    
    intakeButton.and(cone).toggleOnTrue(new IntakeConeCommand(true));

    lowPrepCone.and(cone).onTrue(new AutoDirectionalPrepHeightCommand(TargetLevel.LOW));
    midPrepCone.and(cone).onTrue(new AutoDirectionalPrepHeightCommand(TargetLevel.MID));
    highPrepCone.and(cone).onTrue(new AutoDirectionalPrepHeightCommand(TargetLevel.HIGH, true));

    score.and(cone).toggleOnTrue(new ScoreConeCommand());
    intakeButton.and(cube).toggleOnTrue(new IntakeCubeCommand());

    lowPrepCone.and(cube).onTrue(new CubeAutoDirectionalPrepHeightCommand(TargetLevel.LOW));
    midPrepCone.and(cube).onTrue(new CubeAutoDirectionalPrepHeightCommand(TargetLevel.MID));
    highPrepCone.and(cube).onTrue(new CubeAutoDirectionalPrepHeightCommand(TargetLevel.HIGH));

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
        ArmSubsystem.getInstance().extendNU(Constants.Arm.EXTEND_REST_NU);
      },
      ArmSubsystem.getInstance()
    ));

    eStop.onTrue(new InstantCommand(
      () -> SwerveDriveSubsystem.getInstance().set(0, 0, 0),
      SwerveDriveSubsystem.getInstance()
    ));

    resetPivotNU.onTrue(new InstantCommand(
      () -> ArmSubsystem.getInstance().resetPivotNU()
    ));
  }

  public void configureTabs() {
    Tabs.Intake.add("Intake Test", new IntakeTestCommand(), 0, 0, 2, 1);
    // Tabs.PivotTest.add("Pivot Test", new TestPivotCommand());
    Tabs.Intake.zeroes.add("Extend", new InstantCommand(
      () -> ArmSubsystem.getInstance().zeroExtend(),
      ArmSubsystem.getInstance()
    ));
    // Tabs.Intake.zeroes.add("Pivot",new InstantCommand(
    //   () -> ArmSubsystem.getInstance().zeroPivotSensor(),
    //   ArmSubsystem.getInstance()
    // ));
    Tabs.Intake.zeroes.add("Wrist", new InstantCommand(
      () -> GrabberSubsystem.getInstance().zeroWrist(),
      GrabberSubsystem.getInstance()
    ));
    // Tabs.DriveTest.tab.add(new DriveSpeedCommand());

    // Tabs.GrabberTest.tab.add(new GrabberTestCommand());

    Tabs.Comp.add(new SetPivotOffsetCommand());
  }

  static PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  public static class PDH {
    
    public static PowerDistributionFaults getFaults(){
      return pdh.getFaults();
    }

    //Battery Voltage
    public static double getVoltage() {
      return pdh.getVoltage();
    }

    public static double getTemperature() {
      return pdh.getTemperature();
    }

    //Amps
    //0-23 range for channels
    public static double getCurrent(int channel) {
      return pdh.getCurrent(channel);
    }

    //Watts
    public static double getPower() {
      return pdh.getTotalPower();
    }

    //Joules
    public static double getEnergy() {
      return pdh.getTotalEnergy();
    }
  }
}