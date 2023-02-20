package frc.robot;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.auto.routines.DumbAuto;
import frc.robot.commands.auto.routines.blue.LeftTo0ToScore;
import frc.robot.commands.auto.routines.blue.MidToClimb;
import frc.robot.commands.auto.routines.blue.RightTo3ToScoreAuto;
import frc.robot.commands.auto.routines.red.REDLeftTo0ToScore;
import frc.robot.commands.auto.routines.red.REDMidToClimb;
import frc.robot.commands.auto.routines.red.REDRightTo3ToScoreAuto;
import frc.robot.commands.test.TestAutoCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

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
    autoChooser.setDefaultOption("BLUE-Right2Score", new RightTo3ToScoreAuto());
    autoChooser.addOption("BLUE-Left2Score", new LeftTo0ToScore());
    autoChooser.addOption("BLUE-MidClimb", new MidToClimb());
    autoChooser.setDefaultOption("RED-Right2Score", new REDRightTo3ToScoreAuto());
    autoChooser.addOption("RED-Left2Score", new REDLeftTo0ToScore());
    autoChooser.addOption("RED-MidClimb", new REDMidToClimb());
    autoChooser.addOption("Test", new TestAutoCommand());
    autoChooser.addOption("Dumb Auto", new DumbAuto());
    autoChooser.addOption("Gracious Professionalism", null);

    // SmartDashboard.putData(autoChooser);
    Tabs.Comp.tab.add(autoChooser);

    ArmSubsystem.getInstance().zeroArm();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    // LimelightSubsystem.getInstance().off();
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void autonomousInit() {
    // LimelightSubsystem.getInstance().defaultLED();
    autoChooser.getSelected().schedule();
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // SwerveDriveSubsystem.getInstance().set(0, 0, 0);
    // LimelightSubsystem.getInstance().defaultLED();
    ArmSubsystem.getInstance().stop();
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
