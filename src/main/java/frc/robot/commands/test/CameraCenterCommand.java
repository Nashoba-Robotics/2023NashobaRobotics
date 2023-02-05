package frc.robot.commands.test;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem.TargetType;

/*
Assuming the robot is alread oriented correctly with angle, 
the robot will move horizontally until the April tag is in the middle (centered)

Uses PID control rather than a snapshot
*/
public class CameraCenterCommand extends CommandBase{
    PIDController driveController;
    double maxSpeedPercent = 0.5;
    double setpoint = 0;
    double threshold = 2;

    TargetType t;

    public CameraCenterCommand(){
        addRequirements(SwerveDriveSubsystem.getInstance());

        driveController = new PIDController(0.1, 0, 0);
    }

    // When creating the command, we can tell the robot whether it is trying to center on April Tags or Reflective Tape
    public CameraCenterCommand(TargetType t){
        addRequirements(SwerveDriveSubsystem.getInstance());

        driveController = new PIDController(0.1, 0, 0);
        this.t = t;
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

        driveController.setSetpoint(setpoint);
        driveController.setTolerance(threshold);
    }

    @Override
    public void execute() {
        double tx = LimelightSubsystem.getInstance().getTX();
        double gain = driveController.calculate(tx);

        SmartDashboard.putNumber("Thing: ", gain);
        SmartDashboard.putBoolean("In Range?", driveController.atSetpoint());
        SmartDashboard.putNumber("tx", tx);

        //Switched to field-oriented notation
        SwerveDriveSubsystem.getInstance().set(0, maxSpeedPercent * gain, 0);
    }

    @Override
    public void end(boolean interrupted) {
        SwerveDriveSubsystem.getInstance().set(0, 0, 0);
        LimelightSubsystem.getInstance().setPipeline(Constants.Limelight.APRIL_TAG_PIPELINE);
    }

    @Override
    public boolean isFinished() {
        return !LimelightSubsystem.getInstance().isTarget() || Math.abs(LimelightSubsystem.getInstance().getTX() - setpoint) <= threshold;
    }
}
