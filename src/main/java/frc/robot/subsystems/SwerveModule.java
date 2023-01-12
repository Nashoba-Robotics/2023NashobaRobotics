package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {
    
    public void set(double move, double turn) {

    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(0, Rotation2d.fromDegrees(0));
    }

}
