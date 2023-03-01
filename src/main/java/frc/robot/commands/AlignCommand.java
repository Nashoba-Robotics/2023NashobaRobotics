package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Field.TargetLevel;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem.TargetType;

//TODO: change constants
public class AlignCommand extends CommandBase {
    
    private PIDController turnController;
    private PIDController yController;
    private long startTime;

    TargetType t;
    TargetLevel level;

    public AlignCommand(TargetType t, TargetLevel level) {
        addRequirements(SwerveDriveSubsystem.getInstance(), LimelightSubsystem.getInstance());

        turnController = new PIDController(0, 0, 0);
        yController = new PIDController(0, 0, 0);
        this.t = t;
        this.level = level;
    }

    @Override
    public void initialize() {
        switch(t){
            case APRIL_TAG:
                LimelightSubsystem.getInstance().setPipeline(Constants.Limelight.APRIL_TAG_PIPELINE);
                break;
            case REFLECTIVE_TAPE:
                LimelightSubsystem.getInstance().setPipeline(Constants.Limelight.REFLECTIVE_TAPE_PIPELINE);
                break;
        }

        startTime = System.currentTimeMillis();
        turnController.setSetpoint(0);
        turnController.setTolerance(0);

        yController.setSetpoint(0);
        yController.setTolerance(0);
    }

    @Override
    public void execute() {
        double turnGain = turnController.calculate(-LimelightSubsystem.getInstance().getTX());
        double yGain = yController.calculate(LimelightSubsystem.getInstance().getTX());

        SwerveDriveSubsystem.getInstance().set(0, yGain, turnGain);
    }

    @Override
    public void end(boolean interrupted) {
        SwerveDriveSubsystem.getInstance().set(0, 0, 0);
        LimelightSubsystem.getInstance().setPipeline(Constants.Limelight.APRIL_TAG_PIPELINE);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime > 5000;
    }

}
