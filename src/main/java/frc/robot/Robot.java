package frc.robot;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.auto.intakescore.AutoScoreCubeCommand;
import frc.robot.commands.auto.intakescore.AutoScoreCubePuke;
import frc.robot.commands.auto.intakescore.AutoScoreCubeTest;
import frc.robot.commands.auto.intakescore.AutoScoreTest;
import frc.robot.commands.auto.routines.DumbAuto;
import frc.robot.commands.auto.routines.DumbAutoNoScore;
import frc.robot.commands.auto.routines.LeftTo0ToScore;
import frc.robot.commands.auto.routines.LeftTo0ToScoreToClimb;
import frc.robot.commands.auto.routines.MidToClimb;
import frc.robot.commands.auto.routines.MidToClimbTo1;
import frc.robot.commands.auto.routines.RightTo3ToScoreAuto;
import frc.robot.commands.auto.routines.RightTo3ToScoreToClimb;
import frc.robot.commands.auto.systemcheck.SystemCheck;
import frc.robot.commands.test.IntakeTestCommand;
import frc.robot.commands.test.TestAutoCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.CandleSubsystem.CandleState;

public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  SendableChooser<Command> autoChooser;
  // HttpCamera camera = new HttpCamera("Front Cam", "http://10.17.68.11:5800/");
  public static enum RobotState{
    OK,
    PivotEncoderBad,
    OhSht
  }

  public static RobotState state = RobotState.OK;

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
    autoChooser.addOption("Test", new TestAutoCommand());
    autoChooser.addOption("Dumb Auto", new DumbAuto());
    autoChooser.addOption("Score", new AutoScoreTest());
    autoChooser.addOption("Cube Auto Score", new AutoScoreCubeTest());
    autoChooser.addOption("Cube Puke+", new AutoScoreCubePuke());
    autoChooser.addOption("System Check", new SystemCheck());
    autoChooser.addOption("Gracious Professionalism", new DumbAutoNoScore());
    autoChooser.addOption("BattleCry-CloseScoreClimb", new LeftTo0ToScoreToClimb());
    autoChooser.addOption("BattleCry-FarScoreClimb", new RightTo3ToScoreToClimb());

    Tabs.Comp.tab.add(autoChooser);
    // Tabs.Comp.tab.add("Front Camera", camera);

    Tabs.Intake.tab.add(ArmSubsystem.getInstance());
    Tabs.Intake.tab.add(GrabberSubsystem.getInstance());
    Tabs.Intake.tab.add(new IntakeTestCommand());
    // Tabs.Intake.tab.add("Zero Pivot", new InstantCommand(
    //   () -> ArmSubsystem.getInstance().zeroPivotSensor(),
    //   ArmSubsystem.getInstance()
    // ));
    Tabs.Intake.tab.add("Reset Encoder", new InstantCommand(
      () -> ArmSubsystem.getInstance().resetPivotNU(),
      ArmSubsystem.getInstance()
    ));
    
    ArmSubsystem.getInstance().resetPivotNU();
    // ArmSubsystem.getInstance().zeroPivotSensor();
    SwerveDriveSubsystem.getInstance().resetModulesAbsolute();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // SmartDashboard.putString("Alliance Color", DriverStation.getAlliance().toString());

    if(!ArmSubsystem.getInstance().encoderOK() || RobotContainer.pdh.getFaults().Channel15BreakerFault){
      if(true){
        state = RobotState.PivotEncoderBad;
      }
      else{
        state = RobotState.OhSht;
      }
    }
    else{
      state = RobotState.OK;
    }

    //PDH logging
    // LogManager.appendToLog(RobotContainer.PDH.getVoltage(), "PDH:/Voltage");
    // LogManager.appendToLog(RobotContainer.PDH.getTemperature(), "PDH:/Temperature");
    // LogManager.appendToLog(RobotContainer.PDH.getPower(), "PDH:/Power");
    // LogManager.appendToLog(RobotContainer.PDH.getEnergy(), "PDH:/Energy");
    // for(int i = 0; i < 24; i++) {
    //   LogManager.appendToLog(RobotContainer.PDH.getCurrent(i), "PDH:/Current/"+i);
    // }
    
  }

  @Override
  public void disabledInit() {
    // LimelightSubsystem.getInstance().off();
    // CandleSubsystem.getInstance().set(CandleState.DISABLED);
    // SwerveDriveSubsystem.getInstance().brake();
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
    ArmSubsystem.getInstance().zeroExtend();
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
    LimelightSubsystem.getInstance().setPipeline(Constants.Limelight.REFLECTIVE_TAPE_PIPELINE);

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
