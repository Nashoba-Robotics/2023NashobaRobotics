package frc.robot.lib.math;

import frc.robot.lib.util.SwerveState;

public class SwerveMath {
    
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
