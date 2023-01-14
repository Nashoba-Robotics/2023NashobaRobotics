// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.RunMotorCommand;
import frc.robot.commands.SingleModuleCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.TestCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {

  public static BooleanLogEntry entry;
  public static DataLog log;


  public RobotContainer() {
    // Configure the trigger bindings

    SmartDashboard.putData(new TestCommand());
    SmartDashboard.putData(new SwerveDriveCommand());
    // SmartDashboard.putData(new RunMotorCommand());
    SmartDashboard.putData(new SingleModuleCommand());

    configureBindings();
  }

  private void configureBindings() {
    
  }

}
