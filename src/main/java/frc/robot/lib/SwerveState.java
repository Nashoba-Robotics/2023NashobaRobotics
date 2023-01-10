package frc.robot.lib;

public class SwerveState {

    public double move;
    public double turn; //In radians from -Tau/2(Inclusive) to Tau/2(exclusive)
    public double lastTurn; //Same

    public SwerveState(double move, double turn, double lastTurn) {
        this.move = move;
        this.turn = turn;
        this.lastTurn = lastTurn;
    }
    
}
