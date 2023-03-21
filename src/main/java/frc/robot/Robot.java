package frc.robot;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.auto.intakescore.AutoScoreCommand;
import frc.robot.commands.auto.intakescore.AutoScoreTest;
import frc.robot.commands.auto.move.MoveBackCommand;
import frc.robot.commands.auto.routines.DumbAuto;
import frc.robot.commands.auto.routines.DumbAutoNoScore;
import frc.robot.commands.auto.routines.LeftTo0ToScore;
import frc.robot.commands.auto.routines.MidToClimb;
import frc.robot.commands.auto.routines.MidToClimbTo1;
import frc.robot.commands.auto.routines.RightTo3ToScoreAuto;
import frc.robot.commands.test.IntakeTestCommand;
import frc.robot.commands.test.TestAutoCommand;
import frc.robot.lib.math.NRUnits.Drive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.CandleSubsystem.CandleState;

public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  SendableChooser<Command> autoChooser;
  HttpCamera camera = new HttpCamera("Front Cam", "http://10.17.68.11:5800/");

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // LimelightSubsystem.getInstance().off();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().setDefaultCommand(SwerveDriveSubsystem.getInstance(), new SwerveDriveCommand());
    // SwerveDriveSubsystem.getInstance().set(0, 0, 0);

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("MidClimb+Cube", new MidToClimbTo1());
    autoChooser.addOption("MidClimbOnly", new MidToClimb());
    autoChooser.addOption("Far2Score", new RightTo3ToScoreAuto());
    autoChooser.addOption("Close2Score", new LeftTo0ToScore());
    // autoChooser.addOption("FarClimb", new RightTo3ToBalance());
    // autoChooser.addOption("CloseClimb", new LeftTo0ToBalance());
    autoChooser.addOption("Test", new TestAutoCommand());
    autoChooser.addOption("Dumb Auto", new DumbAuto());
    autoChooser.addOption("Score", new AutoScoreTest());
    autoChooser.addOption("Gracious Professionalism", new DumbAutoNoScore());

    Tabs.Comp.tab.add(autoChooser);
    Tabs.Comp.tab.add("Front Camera", camera);

    Tabs.Intake.tab.add(ArmSubsystem.getInstance());
    Tabs.Intake.tab.add(GrabberSubsystem.getInstance());
    Tabs.Intake.tab.add(new IntakeTestCommand());
    Tabs.Intake.tab.add("Zero Pivot", new InstantCommand(
      () -> ArmSubsystem.getInstance().zeroPivotSensor(),
      ArmSubsystem.getInstance()
    ));
    Tabs.Intake.tab.add("Reset Encoder", new InstantCommand(
      () -> ArmSubsystem.getInstance().resetPivotNU(),
      ArmSubsystem.getInstance()
    ));
    
    ArmSubsystem.getInstance().resetPivotNU();
    ArmSubsystem.getInstance().zeroPivotSensor();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    // LimelightSubsystem.getInstance().off();
    CandleSubsystem.getInstance().set(CandleState.DISABLED);
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void autonomousInit() {
    // LimelightSubsystem.getInstance().defaultLED();
    autoChooser.getSelected().schedule();
    CandleSubsystem.getInstance().set(CandleState.AUTO);
    ArmSubsystem.getInstance().resetPivotNU();
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    SwerveDriveSubsystem.getInstance().set(0, 0, 0);
    // LimelightSubsystem.getInstance().defaultLED();
    ArmSubsystem.getInstance().stop();
    CandleSubsystem.getInstance().set(CandleState.ENABLED);
    ArmSubsystem.getInstance().resetPivotNU();
    CommandScheduler.getInstance().schedule(new SwerveDriveCommand());

    // SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/2);
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    // CandleSubsystem.getInstance().set(CandleState.AUTO);
    CandleSubsystem.getInstance().set(CandleState.WANT_CUBE);
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void simulationInit() {

  }

  @Override
  public void simulationPeriodic() {
    
  }
}
