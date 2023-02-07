package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.test.ArmTestCommand;
import frc.robot.commands.test.BalanceTestCommand;
import frc.robot.commands.test.CameraTestCommand;
import frc.robot.commands.auto.FollowPathCommand;
import frc.robot.commands.test.SwerveDriveTestCommand;

public class RobotContainer {

  public RobotContainer() {
    SmartDashboard.putData(new SwerveDriveCommand());
    SmartDashboard.putData(new SwerveDriveTestCommand());
    SmartDashboard.putData(new FollowPathCommand(PathPlanner.loadPath("testPath", new PathConstraints(4, 2))));
    SmartDashboard.putData(new CameraTestCommand());
    SmartDashboard.putData(new BalanceTestCommand());

    Shuffleboard.getTab("Arm Testing").add(new ArmTestCommand());

    configureBindings();
  }

  private void configureBindings() {
    
  }

}
