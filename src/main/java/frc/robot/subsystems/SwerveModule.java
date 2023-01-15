package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.lib.math.Units;
import frc.robot.lib.util.SwerveState;

public class SwerveModule {
    private TalonFX moveMotor;
    private TalonFX turnMotor;

    private CANCoder turnSensor;

    private int moveMultiplier;

    public SwerveModule(int movePort, int turnPort, int sensorPort, double offset){
        moveMotor = new TalonFX(movePort);
        turnMotor = new TalonFX(turnPort);

        turnSensor = new CANCoder(sensorPort);

        config();
        configOffset(offset); //degrees
        // turnMotor.set(ControlMode.MotionMagic, 0);

    }

    public void config(){
        //Move motor configuration
        moveMotor.configFactoryDefault();
        moveMotor.setNeutralMode(NeutralMode.Brake);
        moveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        moveMotor.setInverted(InvertType.None);

        //Turn motor configuratoin
        turnMotor.configFactoryDefault();
        turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        turnMotor.configNeutralDeadband(0.001);
        turnMotor.config_kF(0, 0.0475);
        turnMotor.config_kP(0, 0.2);
        //turnMotor.config_kI(0, 0.0025);
        turnMotor.config_kD(0, 0.1);
        turnMotor.setInverted(InvertType.InvertMotorOutput);

        int cruiseVelocity = 20_000;
        turnMotor.configMotionCruiseVelocity(cruiseVelocity);
        turnMotor.configMotionAcceleration(2*cruiseVelocity);
        
        turnMotor.configFeedbackNotContinuous(true, 0);

        turnMotor.setNeutralMode(NeutralMode.Coast);
        turnSensor.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        //General Config
        configCurrentLimit();
    }

    public void configCurrentLimit(){
        moveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 45, 0.2));
        moveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.2));

        turnMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 45, 0.2));
        turnMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.2));
    }

    public void configOffset(double offset){
        turnSensor.configMagnetOffset(offset);
        //turnSensor.setPosition(turnSensor.getAbsolutePosition()%180);
        turnMotor.setSelectedSensorPosition(Units.degToNU(turnSensor.getAbsolutePosition()));
    }

    public void zero(){
        set(0, 0);
    }

    public void set(SwerveState state){
        set(state.move, state.turn);
    }
    
    //move input in MPS, Turn input in radians
    public void set(double move, double turn){
        turn *= 180/Math.PI;
        setDeg(move, turn);
    }
    
    //Move input in MPS, Turn input in degrees
    public void setDeg(double move, double turn) {
        double currentPos =  turnMotor.getSelectedSensorPosition();
        double lastTurn = Units.NUToDeg(currentPos);

        double angle = findLowestAngle(turn, lastTurn);
        double angleChange = findAngleChange(angle, lastTurn);
        
        double nextPos = currentPos + Units.degToNU(angleChange);

        turnMotor.set(ControlMode.MotionMagic, nextPos);
        moveMotor.set(ControlMode.PercentOutput, move * moveMultiplier);
    }

    // MPS, Rotation 2D
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Units.NUToMPS(moveMotor.getSelectedSensorVelocity()),
            new Rotation2d(getAngle())
        );
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

    public double findDistance(double turn, double lastTurn){
        double distance = Math.min(Math.abs(turn - lastTurn), Math.abs(turn+360 - lastTurn));
        distance = Math.min(distance, Math.abs(turn - (lastTurn+360)));

        return distance;
    }

    //returns angle in radians
    public double getAngle(){
        return Units.NUToRad(turnMotor.getSelectedSensorPosition());
    }

    //returns CANCoder angle in radians
    public double getAbsAngle(){
        return turnSensor.getAbsolutePosition() * Math.PI/180;
    }

    public double getTurnPosition() {
        return turnMotor.getSelectedSensorPosition();
    }

    public void setTurnMotor(double position) {
        turnMotor.set(ControlMode.MotionMagic, position);
    }
}
