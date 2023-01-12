package frc.robot.lib;
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
    
}
