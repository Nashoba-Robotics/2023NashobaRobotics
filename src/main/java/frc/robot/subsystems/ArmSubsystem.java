package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LogManager;
import frc.robot.Tabs;
import frc.robot.lib.math.NRUnits;

public class ArmSubsystem extends SubsystemBase {
    private TalonFX tromboneSlide;  //Controls the extension/retraction of the arm
    private TalonFX pivot1, pivot2; //Control the pivoting of the entire arm
    private CANCoder encoder;

    public ArmSubsystem(){
        tromboneSlide = new TalonFX(Constants.Arm.ARM_PORT, "drivet");

        pivot1 = new TalonFX(Constants.Arm.PIVOT_PORT_1, "drivet");
        pivot2 = new TalonFX(Constants.Arm.PIVOT_PORT_2, "drivet");

        encoder = new CANCoder(4, "drivet");
        encoder.configMagnetOffset(Constants.Arm.ENCODER_OFFSET);

        config();
    }

    private static ArmSubsystem singleton;
    public static ArmSubsystem getInstance(){
        if(singleton == null) singleton = new ArmSubsystem();
        return singleton;
    }

    public void config(){
        tromboneSlide.configFactoryDefault();
        tromboneSlide.setNeutralMode(NeutralMode.Brake);
        tromboneSlide.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        tromboneSlide.config_kF(0, Constants.Arm.ARM_KF);
        tromboneSlide.config_kP(0, Constants.Arm.ARM_KP);
        tromboneSlide.config_kI(0, Constants.Arm.ARM_KI);
        tromboneSlide.config_kD(0, Constants.Arm.ARM_KD);

        tromboneSlide.configMotionCruiseVelocity(Constants.Arm.ARM_CRUISE_VELOCITY);
        tromboneSlide.configMotionAcceleration(Constants.Arm.ARM_ACCELERATION);

        tromboneSlide.configForwardSoftLimitEnable(true);
        tromboneSlide.configForwardSoftLimitThreshold(Constants.Arm.EXTEND_FORWARD_SOFT_LIMIT);

        tromboneSlide.configReverseSoftLimitEnable(true);
        tromboneSlide.configReverseSoftLimitThreshold(Constants.Arm.EXTEND_REVERSE_SOFT_LIMIT);

        pivot1.configFactoryDefault();
        pivot1.setNeutralMode(NeutralMode.Brake);
        pivot1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        pivot1.config_kF(0, Constants.Arm.PIVOT_KF_1);
        pivot1.config_kP(0, Constants.Arm.PIVOT_KP_1);
        pivot1.config_kI(0, Constants.Arm.PIVOT_KI_1);
        pivot1.config_kD(0, Constants.Arm.PIVOT_KD_1);
        pivot1.configMotionCruiseVelocity(Constants.Arm.PIVOT_CRUISE_VELOCITY);
        pivot1.configMotionAcceleration(Constants.Arm.PIVOT_ACCELERATION);

        pivot2.configFactoryDefault();
        pivot2.setNeutralMode(NeutralMode.Brake);
        pivot2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        pivot2.config_kF(0, Constants.Arm.PIVOT_KF_2);
        pivot2.config_kP(0, Constants.Arm.PIVOT_KP_2);
        pivot2.config_kI(0, Constants.Arm.PIVOT_KI_2);
        pivot2.config_kD(0, Constants.Arm.PIVOT_KD_2);
        pivot2.configMotionCruiseVelocity(Constants.Arm.PIVOT_CRUISE_VELOCITY);
        pivot2.configMotionAcceleration(Constants.Arm.PIVOT_ACCELERATION);

        pivot1.setInverted(InvertType.None);
        pivot2.setInverted(InvertType.InvertMotorOutput);

        pivot1.configForwardSoftLimitEnable(true);
        pivot1.configForwardSoftLimitThreshold(Constants.Arm.PIVOT_FORWARD_SOFT_LIMIT);

        pivot1.configReverseSoftLimitEnable(true);
        pivot1.configReverseSoftLimitThreshold(Constants.Arm.PIVOT_REVERSE_SOFT_LIMIT);

        pivot2.configForwardSoftLimitEnable(true);
        pivot2.configForwardSoftLimitThreshold(Constants.Arm.PIVOT_FORWARD_SOFT_LIMIT);

        pivot2.configReverseSoftLimitEnable(true);
        pivot2.configReverseSoftLimitThreshold(Constants.Arm.PIVOT_REVERSE_SOFT_LIMIT);

        //Positive is extending out
        tromboneSlide.setInverted(InvertType.InvertMotorOutput);
    }

    public void zeroPivot1(){
        pivot1.setSelectedSensorPosition(0);
    }

    public void zeroPivot2(){
        pivot2.setSelectedSensorPosition(0);
    }

    public void zeroPivotSensor(){
        zeroPivot1();
        zeroPivot2();
    }

    //Returns arm to upright position
    public void reset(){
        pivot(0);
    }

    public void stop(){
        set(0);
        setPivot(0);
    }

    public double getPivotOutput(){
        return pivot1.getMotorOutputVoltage();
    }

    //Returns in NU/100ms
    public double getPivotSpeed(){
        double pivotSpeed1 = pivot1.getSelectedSensorVelocity();
        double pivotSpeed2 = pivot2.getSelectedSensorVelocity();

        return (pivotSpeed1+pivotSpeed2)/2;
    }

    //Extends arm to specified position in meters
    public void extendM(double pos){
       tromboneSlide.set(ControlMode.MotionMagic, NRUnits.Extension.mToNU(pos));
    }

    public void extendNU(double nu){
        if(Constants.Logging.ARM) LogManager.appendToLog(nu, "Arm:/Extender/SetNU");
        tromboneSlide.set(ControlMode.MotionMagic, nu);
    }

    //Basic Percent Output set
    public void set(double speed){
       tromboneSlide.set(ControlMode.PercentOutput, speed);
    }

    public boolean extended(){
        //return extendSwitch.get();
        return false;
    }

    public boolean retracted(){
        //return retractSwitch.get();
        return false;
    }

    public double getEncoderAngle(){
        return encoder.getAbsolutePosition();
    }

    public void resetPivotNU(){
        pivot1.setSelectedSensorPosition(NRUnits.Pivot.degToNU(getEncoderAngle()));
        pivot2.setSelectedSensorPosition(NRUnits.Pivot.degToNU(getEncoderAngle()));
    }

    //Pivots arm to specified angle (radians) (0 = upright)
    public void pivot(double angle){
        //How does motion magic work with 2 motors?
        double NU = NRUnits.Pivot.radToNU(angle);
        if(Constants.Logging.ARM) LogManager.appendToLog(NU, "Arm:/Pivot2/SetPosition");

        double ff = 0;
        if(NU >= 8552.632){
            ff = 0.00000076 * tromboneSlide.getSelectedSensorPosition()-0.00653;
        }

        ff *= -Math.sin(angle);

        // pivot1.set(ControlMode.MotionMagic, NU);
        // pivot2.set(ControlMode.MotionMagic, NU);
        
        pivot1.set(ControlMode.MotionMagic, NU, DemandType.ArbitraryFeedForward, ff);
        pivot2.set(ControlMode.MotionMagic, NU, DemandType.ArbitraryFeedForward, ff);
    }

    public void setPivot(double speed){
        pivot1.set(ControlMode.PercentOutput, speed);
        pivot2.set(ControlMode.PercentOutput, speed);
    }

    public void holdPivot(){
        double pivotPos1 = getPivotPos(1);
        pivot1.set(ControlMode.MotionMagic, pivotPos1);

        double pivotPos2 = getPivotPos(2);
        pivot2.set(ControlMode.MotionMagic, pivotPos2);
    }

    public double getExtendNUSpeed(){
        return tromboneSlide.getSelectedSensorVelocity();
    }

    public void holdArm(){
        double armPos = getPos();
        tromboneSlide.set(ControlMode.MotionMagic, armPos);
    }

    //Returns the angle of the arm
    public double getAngle(){
        return (getPivotAngle(1) + getPivotAngle(2))/2;
    }

    public double getExtendNU(){
        return tromboneSlide.getSelectedSensorPosition();
    }

    //Returns the extension of the arm in meters
    public double getLength(){
        double pos = getPos();

        return NRUnits.Extension.NUToM(pos);
    }

    public double getPos(){
        return tromboneSlide.getSelectedSensorPosition();
    }

    public double getArmStatorCurrent() {
        return tromboneSlide.getStatorCurrent();
    }
    
    public double getArmSupplyCurrent() {
        return tromboneSlide.getSupplyCurrent();
    }

    public double getPivotStatorCurrent() {
        return pivot1.getStatorCurrent();
    }

    public double getPivotSupplyCurrent() {
        return pivot1.getSupplyCurrent();
    }

    //This is TEMPORARY
    public double getPivotPos(int n){
        if(n == 1){
            return pivot1.getSelectedSensorPosition();
        }
        else if (n==2){
            return pivot2.getSelectedSensorPosition();
        }
        else return 0;
    }

    //This is also TEMPORARY
    public double getPivotAngle(int n){
        return NRUnits.Pivot.NUToRad(getPivotPos(n));
    }

    //This is TEMPORARY as well
    public double getPivotAngleDeg(int n){
        return getPivotAngle(n) * 360/Constants.TAU;
    }

    public double getStatorCurrent1(){
        return pivot1.getStatorCurrent();
    }

    public double getStatorCurrent2(){
        return pivot2.getStatorCurrent();
    }

    public double getPivotStator(){
        return (getStatorCurrent1()+getStatorCurrent2())/2;
    }

    public double getSupplyCurrent1(){
        return pivot1.getSupplyCurrent();
    }

    public double getSupplyCurrent2(){
        return pivot2.getSupplyCurrent();
    }

    public double getPivotSupply(){
        return (getStatorCurrent1()+getSupplyCurrent2())/2;
    }

    public void setPivotCruiseVelocity(double cruiseVelocity) {
        pivot1.configMotionCruiseVelocity(cruiseVelocity);
        pivot2.configMotionCruiseVelocity(cruiseVelocity);
    }

    public void setPivotAcceleration(double acceleration) {
        pivot1.configMotionAcceleration(acceleration);
        pivot2.configMotionAcceleration(acceleration);
    }

    public void setExtendCruiseVelocity(double cruiseVelocity) {
        tromboneSlide.configMotionCruiseVelocity(cruiseVelocity);
    }

    public void setExtendAcceleration(double acceleration) {
        tromboneSlide.configMotionAcceleration(acceleration);
    }

    public void setDefaultCruiseVelocity() {
        tromboneSlide.configMotionCruiseVelocity(40_000);

        pivot1.configMotionCruiseVelocity(Constants.Arm.ARM_CRUISE_VELOCITY);
        pivot2.configMotionCruiseVelocity(Constants.Arm.ARM_CRUISE_VELOCITY);
    }

    public void setDefaultAcceleration() {
        tromboneSlide.configMotionAcceleration(25_000);

        pivot1.configMotionAcceleration(Constants.Arm.ARM_ACCELERATION);
        pivot2.configMotionAcceleration(Constants.Arm.ARM_ACCELERATION);
    }

    @Override
    public void periodic() {
        if(Constants.Logging.ARM) {
            //Extender
            LogManager.appendToLog(NRUnits.Pivot.NUToRad(tromboneSlide.getSelectedSensorPosition()), "Arm:/Extender/Position");
            LogManager.appendToLog(tromboneSlide.getStatorCurrent(), "Arm:/Extender/Stator");
            LogManager.appendToLog(tromboneSlide.getSupplyCurrent(), "Arm:/Extender/Supply");
            
            //Pivot1
            LogManager.appendToLog(NRUnits.Pivot.NUToRad(pivot1.getSelectedSensorPosition()), "Arm:/Pivot1/Position");
            LogManager.appendToLog(pivot1.getStatorCurrent(), "Arm:/Pivot1/Stator");
            LogManager.appendToLog(pivot1.getSupplyCurrent(), "Arm:/Pivot1/Supply");

            //Pivot2
            LogManager.appendToLog(pivot2.getSelectedSensorPosition(), "Arm:/Pivot2/Position");
            LogManager.appendToLog(pivot2.getStatorCurrent(), "Arm:/Pivot2/Stator");
            LogManager.appendToLog(pivot2.getSupplyCurrent(), "Arm:/Pivot2/Supply");
        }

        SmartDashboard.putNumber("PivotCurrent", pivot1.getStatorCurrent());

        Tabs.Comp.displayPivotAngle(getAngle());
        Tabs.Comp.displayEncoderAngle(getEncoderAngle());
        Tabs.Comp.displayExtendNU(getExtendNU());
    }
}