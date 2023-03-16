package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class CameraTestCommand extends CommandBase{
    double tagHeight = 0.43;
    double camHeight = 0.515;

    double mod1Pos;
    public CameraTestCommand() {
        addRequirements(SwerveDriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        LimelightSubsystem.getInstance().setPipeline(Constants.Limelight.APRIL_TAG_PIPELINE);
        mod1Pos = SwerveDriveSubsystem.getInstance().getMod1NU();
    }

    @Override
    public void execute() {
        double tx = (LimelightSubsystem.getInstance().getTX()+1.5) * Constants.TAU/360;
        double ty = LimelightSubsystem.getInstance().getTY() * Constants.TAU/360;

        SmartDashboard.putNumber("x", tx);
        SmartDashboard.putNumber("y", ty);

        double vDist = (camHeight-tagHeight)/Math.tan(Math.abs(ty));
        SmartDashboard.putNumber("Vert", vDist);

        double hyp = vDist/Math.cos(Math.abs(tx));
        SmartDashboard.putNumber("Hyp", hyp);

        SwerveDriveSubsystem.getInstance().setAngle(tx);
        SwerveDriveSubsystem.getInstance().driveNU(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
