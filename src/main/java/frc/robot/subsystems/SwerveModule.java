package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.lib.Units;
import frc.robot.lib.SwerveState;

public class SwerveModule {
    private TalonFX moveMotor;
    private TalonFX turnMotor;

    private CANCoder turnSensor;

    private double offset;
    private int moveMultiplier;

    public SwerveModule(int movePort, int turnPort, int sensorPort, double offset){
        moveMotor = new TalonFX(movePort);
        turnMotor = new TalonFX(turnPort);

        turnSensor = new CANCoder(sensorPort);

        this.offset = offset;
    }

    public void set(SwerveState state){
        set(state.move, state.turn);
    }
    
    public void set(double move, double turn){
        turn *= 180/Math.PI;
        setDeg(move, turn);
    }
    
    //Input is radians
    public void setDeg(double move, double turn) {
        double currentPos =  turnMotor.getSelectedSensorPosition();
        double lastTurn = Units.NUToDeg(currentPos);

        double angle = findLowestAngle(turn, lastTurn);
        double angleChange = findAngleChange(angle, lastTurn);
        
        double nextPos = currentPos + Units.degToNU(angleChange);

        turnMotor.set(ControlMode.MotionMagic, nextPos);
    }

    public SwerveModulePosition getPosition() {
        return null;
    }

    public double findLowestAngle(double turn, double lastTurn){
        double[] potAngles = potentialAngles(turn); //Gets the two potential angles we could go to

        // Calculate the distance between those and the last angle the module was at
        double originalDistance = findDistance(potAngles[0], lastTurn);
        double oppositeDistance = findDistance(potAngles[1], lastTurn);

        // If the original distance is less, we want to go there
        if(originalDistance <= oppositeDistance){
            moveMultiplier = 1;
            return potAngles[0];
        }
        else{
            moveMultiplier = -1;
            return potAngles[1];
        } 
    }

    // Find the two angles we could potentially go to
    public double[] potentialAngles(double angle){
        //Constrain the variable to desired domain
        angle = Units.constrainDeg(angle);
        //Figure out the opposite angle
        double oppositeAngle = angle + 180;
        //Constrain the opposite angle
        oppositeAngle = Units.constrainDeg(oppositeAngle);
        //Put them into a size 2 array
        double[] angles = {angle, oppositeAngle};
        //return it
        return angles;
    }

    public double findAngleChange(double turn, double lastTurn){
        double distance = turn - lastTurn;
        //double sign = Math.signum(distance);   //Either 1 or -1 -> represents positive or negative

        if(Math.abs(turn - (lastTurn + 360)) < Math.abs(distance)){
            // If this is true, it means that lastTurn is in the negatives and is trying to reach a positive, meaning that it must move positive
            distance = turn - (lastTurn + 360);
            //sign = +1;
        }

        if(Math.abs(turn+360 - (lastTurn)) < Math.abs(distance)){
            // If this is true, it means that turn is in the negatives and lastTurn is trying to reach a negative, meaning that you must move negative 
            distance = turn+360 - lastTurn;
            //sign = -1;
        }

        return distance;
    }

    double findDistance(double turn, double lastTurn){
        double distance = Math.min(Math.abs(turn - lastTurn), Math.abs(turn+360 - lastTurn));
        distance = Math.min(distance, Math.abs(turn - (lastTurn+360)));

        return distance;
    }
}
