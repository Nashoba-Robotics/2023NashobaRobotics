// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.test.BalanceTestCommand;
import frc.robot.commands.auto.TestAutoCommand;
import frc.robot.commands.test.RunMotorCommand;
import frc.robot.commands.test.SingleModuleCommand;
import frc.robot.commands.test.SwerveDriveTestCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {

  public static BooleanLogEntry entry;
  public static DataLog log;

  private static CommandBase autoCommand;


  public RobotContainer() {
    // Configure the trigger bindings

    SmartDashboard.putData(new SwerveDriveCommand());
    SmartDashboard.putData(new SwerveDriveTestCommand());
    SmartDashboard.putData(getAutoCommand());

    SmartDashboard.putData(new BalanceTestCommand());

    configureBindings();
  }

  private void configureBindings() {
    
  }

  public static CommandBase getAutoCommand() {
    if(autoCommand == null) autoCommand = new TestAutoCommand();
    return autoCommand;
  }

}
