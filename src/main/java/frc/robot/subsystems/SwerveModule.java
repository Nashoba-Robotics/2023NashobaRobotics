package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.CANcoderConfigurator;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.math.NRUnits;
import frc.robot.lib.util.SwerveState;

public class SwerveModule {
    public int modNumber;

    private TalonFX moveMotor;
    private TalonFXConfigurator moveConfigurator;
    private TalonFXConfiguration moveConfig;

    private VelocityDutyCycle moveControl;

    private TalonFX turnMotor;
    private TalonFXConfigurator turnConfigurator;
    private TalonFXConfiguration turnConfig;

    private MotionMagicDutyCycle turnControl;

    private CANcoder turnSensor;
    private CANcoderConfigurator turnSensorConfigurator;
    private CANcoderConfiguration sensorConfig;

    private double movePosition;
    private double lastMovePosition;

    private double AFF;

    public SwerveModule(int modNumber, int movePort, int turnPort, int sensorPort, double offset, double AFF){
        this.AFF = AFF;
        this.modNumber = modNumber;

        moveMotor = new TalonFX(movePort, "drivet");    //Why is it called drivet?
        moveConfigurator = moveMotor.getConfigurator();
        moveConfig = new TalonFXConfiguration();

        moveControl = new VelocityDutyCycle(0);
        moveControl.FeedForward = AFF;

        turnMotor = new TalonFX(turnPort, "drivet");    //It has the DRIVE and PIVET!!
        turnConfigurator = turnMotor.getConfigurator();
        turnConfig = new TalonFXConfiguration();

        turnControl = new MotionMagicDutyCycle(offset);

        turnSensor = new CANcoder(sensorPort, "drivet");
        turnSensorConfigurator = turnSensor.getConfigurator();
        sensorConfig = new CANcoderConfiguration();


        config();
        configOffset(offset); //NU

        movePosition = 0;
        lastMovePosition = 0;
    }

    public void config(){
        //Move motor configuration
        moveConfig.Audio.BeepOnBoot = true;
        moveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        moveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        moveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        moveConfig.Slot0.kV = Constants.Swerve.MOVE_KF;
        moveConfig.Slot0.kS = AFF;
        moveConfig.Slot0.kP = Constants.Swerve.MOVE_KP;
        moveConfig.Slot0.kI = Constants.Swerve.MOVE_KI;
        moveConfig.Slot0.kD = Constants.Swerve.MOVE_KD;
        moveConfig.Voltage.PeakForwardVoltage = 12;
        moveConfig.Voltage.PeakReverseVoltage = -12;
        moveConfig.CurrentLimits.StatorCurrentLimit = 40;
        moveConfig.CurrentLimits.SupplyCurrentLimit = 60;
        moveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        moveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        moveConfigurator.apply(moveConfig);
        moveMotor.setRotorPosition(0);

        //Turn motor configuratoin
        turnConfig.Audio.BeepOnBoot = true;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turnConfig.Slot0.kV = Constants.Swerve.TURN_KF;
        turnConfig.Slot0.kP = Constants.Swerve.TURN_KP;
        turnConfig.Slot0.kI = Constants.Swerve.TURN_KI;
        turnConfig.Slot0.kD = Constants.Swerve.TURN_KD;
        turnConfig.Voltage.PeakForwardVoltage = 12;
        turnConfig.Voltage.PeakReverseVoltage = -12;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity = 103;
        turnConfig.MotionMagic.MotionMagicAcceleration = 180;
        turnConfig.MotionMagic.MotionMagicJerk = 0;
        turnConfig.CurrentLimits.StatorCurrentLimit = 40;
        turnConfig.CurrentLimits.SupplyCurrentLimit = 60;
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfigurator.apply(turnConfig);

        sensorConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        sensorConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        turnSensorConfigurator.apply(sensorConfig);
    }

    public void configOffset(double offset){
        sensorConfig.MagnetSensor.MagnetOffset = offset;
        turnSensorConfigurator.apply(sensorConfig);
        turnMotor.setRotorPosition(NRUnits.Drive.degToNU(turnSensor.getAbsolutePosition().getValue()*360));
    }

    public void resetTurnToAbsolute() {
        turnMotor.setRotorPosition(NRUnits.Drive.degToNU(turnSensor.getAbsolutePosition().getValue()*360));
    }

    public void zero(){
        set(0, 0);
    }

    public void set(SwerveState state){
        set(state.move, state.turn);
    }

    public void set(SwerveModuleState state){
        set(NRUnits.Drive.toPercentOutput(state.speedMetersPerSecond), state.angle.getRadians());
    }
    
    //move input in percent, Turn input in radians
    public void set(double move, double turn){
        turn *= 360/Constants.TAU;
        setDeg(move, turn);
    }

    public void set(double move, double turn, boolean optimizeTurn){
        turn *= 360/Constants.TAU;
        setDeg(move, turn, optimizeTurn);
    }
    
    //Move input in percent, Turn input in degrees
    public void setDeg(double move, double turn) {
        if(move == 0){
            moveMotor.set(0);
            return;
        }
        double currentPos =  turnMotor.getRotorPosition().getValue();
        double lastTurn = NRUnits.constrainDeg(NRUnits.Drive.NUToDeg(currentPos));

        double angle = findLowestAngle(turn, lastTurn);
        double angleChange = findAngleChange(angle, lastTurn);
        
        double nextPos = currentPos + NRUnits.Drive.degToNU(angleChange);

        SmartDashboard.putNumber("ActualAngle"+modNumber, getAngle() * Constants.TAU / 360);

        turnControl.Position = nextPos;
        turnMotor.setControl(turnControl);
        moveControl.Velocity = move * Constants.Swerve.MAX_NATIVE_VELOCITY;
        moveMotor.setControl(moveControl);
    }

    public void setDeg(double move, double turn, boolean optimizeTurn) {
        if(move == 0 && optimizeTurn){
            moveMotor.set(0);
            return;
        }
        double currentPos =  turnMotor.getRotorPosition().getValue();
        double lastTurn = NRUnits.constrainDeg(NRUnits.Drive.NUToDeg(currentPos));

        double angle = findLowestAngle(turn, lastTurn);
        double angleChange = findAngleChange(angle, lastTurn);
        
        double nextPos = currentPos + NRUnits.Drive.degToNU(angleChange);

        SmartDashboard.putNumber("ActualAngle"+modNumber, getAngle() * Constants.TAU / 360);

        turnControl.Position = nextPos;
        turnMotor.setControl(turnControl);
        moveControl.Velocity = move * Constants.Swerve.MAX_NATIVE_VELOCITY;
        moveControl.FeedForward = AFF;
        moveMotor.setControl(moveControl);
    }

    //radian input
    public void turn(double turn){
        turn *= 360/Constants.TAU;
        double currentPos =  turnMotor.getRotorPosition().getValue();
        double lastTurn = NRUnits.constrainDeg(NRUnits.Drive.NUToDeg(currentPos));

        double angle = findLowestAngle(turn, lastTurn);
        double angleChange = findAngleChange(angle, lastTurn);
        
        double nextPos = currentPos + NRUnits.Drive.degToNU(angleChange);

        turnControl.Position = nextPos;
        turnMotor.setControl(turnControl);
    }

    //NU: How many ADDITIONAL NU you want to move   turn: radians
    public void moveNUDeg(double NU, double turn){
        // turn(turn);
        setMovePos(NU);
    }

    // MPS, Rotation 2D
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            NRUnits.Drive.NUToM(movePosition),
            Rotation2d.fromRadians(moveMotor.getInverted() ?  NRUnits.constrainRad(getAbsAngle()+Constants.TAU/2) : getAbsAngle())
        );
    }

    public double findLowestAngle(double turn, double lastTurn){
        double[] potAngles = potentialAngles(turn); //Gets the two potential angles we could go to

        // Calculate the distance between those and the last angle the module was at
        double originalDistance = findDistance(potAngles[0], lastTurn);
        double oppositeDistance = findDistance(potAngles[1], lastTurn);

        // If the original distance is less, we want to go there
        if(originalDistance <= oppositeDistance){
            moveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            moveConfigurator.apply(moveConfig);
            return potAngles[0];
        }
        else{ //If we want to go to the opposite of the desired angle, we have to tell the motor to move "backwards"
            moveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            moveConfigurator.apply(moveConfig);
            return potAngles[1];
        } 
    }

    // Find the two angles we could potentially go to
    public double[] potentialAngles(double angle){
        //Constrain the variable to desired domain
        angle = NRUnits.constrainDeg(angle);

        //Figure out the opposite angle
        double oppositeAngle = angle + 180;

        //Constrain the opposite angle
        oppositeAngle = NRUnits.constrainDeg(oppositeAngle);

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
        return NRUnits.Drive.NUToRad(turnMotor.getRotorPosition().getValue() * 360);
    }

    //returns CANCoder angle in radians
    public double getAbsAngle(){
        return turnSensor.getAbsolutePosition().getValue() * Constants.TAU;
    }

    public double getTurnPosition() {
        return turnMotor.getPosition().getValue();
    }

    public double getTurnAngle(){
        return NRUnits.constrainRad(getTurnPosition() * Constants.TAU);
    }

    // RADIANS
    public boolean atTargetAngle(double targetAngle){
        double deadzone = Constants.TAU/12;
        return Math.abs(getTurnAngle() - targetAngle) < deadzone;
    }

    public void setTurnMotor(double position) {
        turnControl.Position = position;
        turnMotor.setControl(turnControl);
    }

    public void resetTurnPosition() {
        turnMotor.setRotorPosition(0);
    }

    public double getMovePosition() {
        return movePosition;
    }

    public double getMoveVelocity() {
        return moveMotor.getVelocity().getValue();
    }

    public void setMoveVelocity(double move) {
        moveControl.Velocity = move * Constants.Swerve.MAX_NATIVE_VELOCITY;
        moveControl.FeedForward = AFF;
        moveMotor.setControl(moveControl);
    }

    public void updateMovePosition() {
        double temp = Math.abs(moveMotor.getPosition().getValue());
        movePosition += Math.abs(temp - lastMovePosition);
        lastMovePosition = temp;
    }

    public void setMovePos(double NU){
        moveConfig.MotionMagic.MotionMagicCruiseVelocity = 20_000;
        moveConfig.MotionMagic.MotionMagicAcceleration = 8_000;
        moveConfigurator.apply(moveConfig);
        
        double currPos = moveMotor.getPosition().getValue();
        turnControl.Position = currPos + NU;
        turnMotor.setControl(turnControl);
    }

    public void setMoveSpeed(double s){
        moveMotor.set(s);
    }

    public double getNU(){
        return moveMotor.getPosition().getValue();
    }
    
    public SwerveModuleState getSwerveState() {
        return new SwerveModuleState(
                NRUnits.Drive.NUToMPS(moveMotor.getVelocity().getValue()),
                // Rotation2d.fromRadians(NRUnits.Drive.NUToRad(turnMotor.getSelectedSensorPosition()))
                Rotation2d.fromRadians(getAbsAngle())
            );
    }

    public double getMoveSensorNU() {
        return moveMotor.getPosition().getValue();
    }

    public boolean encoderOK(){
        // CANCoderFaults faults = new CANCoderFaults();
        // turnSensor.getFaults(faults);

        // if(faults.APIError) return false;
        // if(faults.HardwareFault) return false;
        // if(faults.MagnetTooWeak) return false;
        // if(faults.ResetDuringEn) return false;
        // if(faults.UnderVoltage) return false;

        // if(turnSensor.getLastError() != ErrorCode.OK) return false;

        return true;
    }
}
