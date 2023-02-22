package frc.robot;

import javax.swing.text.StyleContext.SmallAttributeSet;

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
import frc.robot.commands.auto.routines.DumbAuto;
import frc.robot.commands.auto.routines.LeftTo0ToScore;
import frc.robot.commands.auto.routines.MidToClimb;
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
    autoChooser.setDefaultOption("Right2Score", new RightTo3ToScoreAuto());
    autoChooser.addOption("Left2Score", new LeftTo0ToScore());
    autoChooser.addOption("MidClimb", new MidToClimb());
    autoChooser.addOption("Test", new TestAutoCommand());
    autoChooser.addOption("Dumb Auto", new DumbAuto());
    autoChooser.addOption("Gracious Professionalism", null);

    // SmartDashboard.putData(autoChooser);
    Tabs.Comp.tab.add(autoChooser);

    Tabs.Intake.tab.add(ArmSubsystem.getInstance());
    Tabs.Intake.tab.add(GrabberSubsystem.getInstance());
    Tabs.Intake.tab.add(new IntakeTestCommand());
    Tabs.Intake.tab.add("Zero Pivot", new InstantCommand(
      () -> ArmSubsystem.getInstance().zeroPivotSensor(),
      ArmSubsystem.getInstance()
    ));
    Tabs.Intake.tab.add("Zero Arm", new InstantCommand(
      () -> ArmSubsystem.getInstance().zeroArmSensor(),
      ArmSubsystem.getInstance()
    ));
    ArmSubsystem.getInstance().zeroArmSensor();
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
    // SwerveDriveSubsystem.getInstance().setGyro(Constants.TAU/4);
    autoChooser.getSelected().schedule();
    CandleSubsystem.getInstance().set(CandleState.AUTO);
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // SwerveDriveSubsystem.getInstance().set(0, 0, 0);
    // LimelightSubsystem.getInstance().defaultLED();
    ArmSubsystem.getInstance().stop();
    CandleSubsystem.getInstance().set(CandleState.ENABLED);
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
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
