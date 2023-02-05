package frc.robot.commands.test;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class FieldOrientCommand extends CommandBase {

    private PIDController controller;
    private long startTime;
    
    public FieldOrientCommand() {
        PIDController controller = new PIDController(0, 0, 0);
        addRequirements(SwerveDriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        SwerveDriveSubsystem.getInstance().set(0, 0, controller.calculate(SwerveDriveSubsystem.getInstance().getGyroAngle(), Constants.TAU/4));
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint()
            || System.currentTimeMillis() - startTime > 1000;
    }

}
