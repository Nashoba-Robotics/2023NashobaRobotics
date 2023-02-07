package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.lib.math.Units;
import frc.robot.lib.util.SwerveState;

public class SwerveModule {
    public int modNumber;

    private TalonFX moveMotor;
    private TalonFX turnMotor;

    private CANCoder turnSensor;

    private double movePosition;
    private double lastMovePosition;

    private double AFF;

    public SwerveModule(int modNumber, int movePort, int turnPort, int sensorPort, double offset, double AFF){
        this.AFF = AFF;
        this.modNumber = modNumber;

        moveMotor = new TalonFX(movePort);
        turnMotor = new TalonFX(turnPort);

        turnSensor = new CANCoder(sensorPort);

        config();
        configOffset(offset); //degrees

        movePosition = 0;
        lastMovePosition = 0;
    }

    public void config(){
        //Move motor configuration
        moveMotor.configFactoryDefault();
        moveMotor.setNeutralMode(NeutralMode.Brake);
        moveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        moveMotor.setSelectedSensorPosition(0);
        moveMotor.setInverted(InvertType.None);
        moveMotor.config_kF(0, Constants.Swerve.MOVE_KF);
        moveMotor.config_kP(0, Constants.Swerve.MOVE_KP);
        moveMotor.config_kI(0, Constants.Swerve.MOVE_KI);
        moveMotor.config_kD(0, Constants.Swerve.MOVE_KD);

        //Turn motor configuratoin
        turnMotor.configFactoryDefault();
        turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        turnMotor.configNeutralDeadband(0.001);
        turnMotor.config_kF(0, Constants.Swerve.TURN_KF);
        turnMotor.config_kP(0, Constants.Swerve.TURN_KP);
        turnMotor.config_kI(0, Constants.Swerve.TURN_KI);
        turnMotor.config_kD(0, Constants.Swerve.TURN_KD);
        turnMotor.setInverted(InvertType.InvertMotorOutput);

        int cruiseVelocity = 22_000;
        turnMotor.configMotionCruiseVelocity(cruiseVelocity);
        turnMotor.configMotionAcceleration(2*cruiseVelocity);
        
        //turnMotor.configFeedbackNotContinuous(true, 0);
        turnMotor.setNeutralMode(NeutralMode.Brake);

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
        turnMotor.setSelectedSensorPosition(Units.Drive.degToNU(turnSensor.getAbsolutePosition()));
    }

    public void zero(){
        set(0, 0);
    }

    public void set(SwerveState state){
        set(state.move, state.turn);
    }

    public void set(SwerveModuleState state){
        set(Units.Drive.toPercentOutput(state.speedMetersPerSecond), state.angle.getRadians());
    }
    
    //move input in percent, Turn input in radians
    public void set(double move, double turn){
        turn *= 180/Math.PI;
        setDeg(move, turn);
    }
    
    //Move input in percent, Turn input in degrees
    public void setDeg(double move, double turn) {
        if(move == 0){
            moveMotor.set(ControlMode.PercentOutput, 0);
            return;
        }
        double currentPos =  turnMotor.getSelectedSensorPosition();
        double lastTurn = Units.constrainDeg(Units.Drive.NUToDeg(currentPos));

        double angle = findLowestAngle(turn, lastTurn);
        double angleChange = findAngleChange(angle, lastTurn);
        
        double nextPos = currentPos + Units.Drive.degToNU(angleChange);

        turnMotor.set(ControlMode.MotionMagic, nextPos);
        moveMotor.set(ControlMode.Velocity, move * Constants.Swerve.MAX_NATIVE_VELOCITY, DemandType.ArbitraryFeedForward, AFF);
    }

    // MPS, Rotation 2D
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Units.Drive.NUToM(movePosition),
            Rotation2d.fromRadians(moveMotor.getInverted() ?  Units.constrainRad(getAbsAngle()+Constants.TAU/2) : getAbsAngle())
        );
    }

    public double findLowestAngle(double turn, double lastTurn){
        double[] potAngles = potentialAngles(turn); //Gets the two potential angles we could go to

        // Calculate the distance between those and the last angle the module was at
        double originalDistance = findDistance(potAngles[0], lastTurn);
        double oppositeDistance = findDistance(potAngles[1], lastTurn);

        // If the original distance is less, we want to go there
        if(originalDistance <= oppositeDistance){
            moveMotor.setInverted(InvertType.None);
            return potAngles[0];
        }
        else{ //If we want to go to the opposite of the desired angle, we have to tell the motor to move "backwards"
            moveMotor.setInverted(InvertType.InvertMotorOutput);
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
        return Units.Drive.NUToRad(turnMotor.getSelectedSensorPosition());
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

    public void resetTurnPosition() {
        turnMotor.setSelectedSensorPosition(0);
    }

    public double getMovePosition() {
        return movePosition;
    }

    public double getMoveVelocity() {
        return moveMotor.getSelectedSensorVelocity();
    }

    public void setMoveVelocity(double move) {
        moveMotor.set(ControlMode.Velocity, move);
    }

    public void updateMovePosition() {
        double temp = Math.abs(moveMotor.getSelectedSensorPosition());
        movePosition += Math.abs(temp - lastMovePosition);
        lastMovePosition = temp;
    }
}
