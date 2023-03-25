package frc.robot.lib.math;

import frc.robot.lib.util.SwerveState;

public class SwerveMath {
    
    //if magnitudes are greater than 1, will normalize all the states so that the max is 1
    public static SwerveState[] normalize(SwerveState[] states) {
        double max = 0;
        for(SwerveState state : states) {
            max = state.move > max ? state.move : max;
        }

        if(max > 1) {
            for(SwerveState state : states) {
                state.move /= max;
            }
        }

        return states;
    }

}
