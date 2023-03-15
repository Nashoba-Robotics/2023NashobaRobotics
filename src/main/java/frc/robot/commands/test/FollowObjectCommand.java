package frc.robot.commands.test;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.JoystickSubsytem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class FollowObjectCommand extends CommandBase {

    PIDController strafeController;
    
    public FollowObjectCommand() {
        addRequirements(SwerveDriveSubsystem.getInstance());

        strafeController = new PIDController(0.01, 0, 0);
        strafeController.setSetpoint(0);
        strafeController.setTolerance(5);
    }

    @Override
    public void initialize() {
        SwerveDriveSubsystem.getInstance().setFieldCentric(false);
        LimelightSubsystem.getInstance().setPipeline(3);
    }

    @Override
    public void execute() {
        
        double ty = LimelightSubsystem.getInstance().getTY();
        double gain = strafeController.calculate(ty);

        SwerveDriveSubsystem.getInstance().set(JoystickSubsytem.getInstance().getLeftJoystickValues().x, -gain, 0);

        SmartDashboard.putNumber("ty", ty);

    }

    @Override
    public void end(boolean interrupted) {
        SwerveDriveSubsystem.getInstance().set(0, 0, 0);
        SwerveDriveSubsystem.getInstance().setFieldCentric(true);
    }

}
