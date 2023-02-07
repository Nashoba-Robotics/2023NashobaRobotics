package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;

public class CarpetOdometry extends SwerveDriveOdometry {

    private Rotation2d angOfResistance;
    private SwerveModulePosition[] currPositions;
    private double[] lastPositions;
    public static int count = 0;

    public CarpetOdometry(SwerveDriveKinematics kinematics, Rotation2d rotation, SwerveModulePosition[] positions, Rotation2d angOfResistance) {
        super(kinematics, rotation, positions);
        this.angOfResistance = angOfResistance;
        lastPositions = new double[positions.length];
        currPositions = new SwerveModulePosition[positions.length];
        for(int i = 0; i < positions.length; i++) {
            lastPositions[i] = positions[i].distanceMeters;
        }
        for(int i = 0; i < positions.length; i++) {
            currPositions[i] = new SwerveModulePosition(0, positions[i].angle);
        }
    }

    @Override
    public void resetPosition(Rotation2d rotation, SwerveModulePosition[] positions, Pose2d pose) {
        for(int i = 0; i < positions.length; i++) {
            lastPositions[i] = positions[i].distanceMeters;
        }
        for(int i = 0; i < positions.length; i++) {
            currPositions[i] = new SwerveModulePosition(0, positions[i].angle);
        }
        super.resetPosition(rotation, currPositions, pose);
    }

    @Override
    public Pose2d update(Rotation2d rotation, SwerveModulePosition[] positions) {
        double[] deltas = new double[positions.length];
        for(int i = 0; i < deltas.length; i++) {
            deltas[i] = positions[i].distanceMeters - lastPositions[i];
        }
        for(int i = 0; i < deltas.length; i++) {
            deltas[i] *= 1 - Constants.Field.K_CARPET * Math.cos(positions[i].angle.getRadians() + super.getPoseMeters().getRotation().getRadians() - angOfResistance.getRadians()) + Constants.Field.K_CARPET;
        }
        for(int i = 0; i < positions.length; i++) {
            lastPositions[i] = positions[i].distanceMeters;
        }
        for(int i = 0; i < currPositions.length; i++) {
            currPositions[i].distanceMeters += deltas[i];
            currPositions[i].angle = positions[i].angle;
        }

        return super.update(rotation, currPositions);
    }

}
