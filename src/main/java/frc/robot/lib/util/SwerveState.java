package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Translation2d;

/*
    stores two values used to set a swerve state:
        move [-1, 1] : speed of the motor for swerve movement
        turn [-Tau/2, Tau/2) : angle for turn motor
 */
public class SwerveState {

    public double move;
    public double turn;

    public SwerveState(double move, double turn) {
        this.move = move;
        this.turn = turn;
    }

    public static SwerveState fromTranslation2d(Translation2d translation) {
        return new SwerveState(translation.getDistance(new Translation2d(0, 0)), translation.getAngle().getRadians());
    }

    public static SwerveState[] fromTranslation2d(Translation2d[] translations) {
        SwerveState[] states = new SwerveState[translations.length];
        int i = 0;
        for(Translation2d translation : translations) {
            states[i] = fromTranslation2d(translation);
            i++;
        }
        return states;
    }
    
}
