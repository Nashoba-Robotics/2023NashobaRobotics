package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class FieldLocations {
 
    public final static class Blue {
        public final static double SCORE_X = 1.85;
        public final static double INTAKE_X = 6.53;

        public final static Translation2d LEFT_A = new Translation2d(SCORE_X, 4.96);
        public final static Translation2d LEFT_B = new Translation2d(SCORE_X, 0);
        public final static Translation2d LEFT_C = new Translation2d(SCORE_X, 3.85);

        public final static Translation2d MID_A = new Translation2d(SCORE_X, 0);
        public final static Translation2d MID_B = new Translation2d(SCORE_X, 2.73);
        public final static Translation2d MID_C = new Translation2d(SCORE_X, 0);

        public final static Translation2d RIGHT_A = new Translation2d(SCORE_X, 1.60);
        public final static Translation2d RIGHT_B = new Translation2d(SCORE_X, 0);
        public final static Translation2d RIGHT_C = new Translation2d(SCORE_X, 0.50);

        public final static Translation2d PIECE_0 = new Translation2d(INTAKE_X, 4.64);
        public final static Translation2d PIECE_1 = new Translation2d(INTAKE_X, 3.38);
        public final static Translation2d PIECE_2 = new Translation2d(INTAKE_X, 2.16);
        public final static Translation2d PIECE_3 = new Translation2d(INTAKE_X, 0.92);

        public final static Translation2d INNER_BALANCE = new Translation2d(0, 0);
        public final static Translation2d OUTER_BALANCE = new Translation2d(0, 0);
    }

    public final static class Red {

    }
    
}