// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.test.ArmTestCommand;
import frc.robot.commands.test.BalanceTestCommand;
import frc.robot.commands.test.CameraTestCommand;
import frc.robot.commands.auto.FollowPathCommand;
import frc.robot.commands.test.SwerveDriveTestCommand;

public class RobotContainer {

  public static BooleanLogEntry entry;
  public static DataLog log;

  private static CommandBase autoCommand;


  public RobotContainer() {
    // Configure the trigger bindings

    SmartDashboard.putData(new SwerveDriveCommand());
    SmartDashboard.putData(new SwerveDriveTestCommand());
    SmartDashboard.putData(getAutoCommand());
    SmartDashboard.putData(new CameraTestCommand());

    SmartDashboard.putData(new BalanceTestCommand());

    Shuffleboard.getTab("Arm Testing").add(new ArmTestCommand());

    configureBindings();
  }

  private void configureBindings() {
    
  }

  public static CommandBase getAutoCommand() {
    if(autoCommand == null) autoCommand = new FollowPathCommand(PathPlanner.loadPath("testPath", new PathConstraints(4, 2)));
    return autoCommand;
  }

}
