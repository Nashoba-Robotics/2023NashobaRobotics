package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LogManager;

public class LoggingTestCommand extends CommandBase {
    @Override
    public void initialize() {
        LogManager.logMessage("Test command started");
        LogManager.appendToLog("COMMAND_START_VALUE", "test:/commandStart");
    }

    @Override
    public void execute() {
        LogManager.logMessage("Test command continuing");
        LogManager.appendToLog("COMMAND_CONTINUE_VALUE", "test:/commandCont");
    }

    @Override
    public void end(boolean interrupted) {
        LogManager.logMessage("Test command ended");
        LogManager.appendToLog("COMMAND_END_VALUE", "test:/commandEnd");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
