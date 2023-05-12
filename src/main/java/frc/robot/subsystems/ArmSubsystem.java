package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LogManager;
import frc.robot.Tabs;
import frc.robot.lib.math.NRUnits;

public class ArmSubsystem extends SubsystemBase {
    private TalonFX tromboneSlide;  //Controls the extension/retraction of the arm
    private TalonFXConfigurator tuningSlide;
    private TalonFXConfiguration defaultTune;
    private TalonFXConfiguration tuningConfig;
    private CANCoder encoder;
    private MotionMagicDutyCycle posSetter;

    

    private TalonFX kick1, kick2; //Control the pivoting of the entire arm
    private TalonFXConfigurator foot1, foot2;
    private TalonFXConfiguration defaultFoot;

    public ArmSubsystem(){
        tromboneSlide = new TalonFX(Constants.Arm.ARM_PORT, "drivet");
        tuningSlide = tromboneSlide.getConfigurator();
        posSetter = new MotionMagicDutyCycle(0);    //<-- Do I need to call setControl() every time I change this?
                                                             //     If I change the value of posSetter, will it change without a new call to setControl()?
        // tuningConfig = new TalonFXConfiguration();

        kick1 = new TalonFX(Constants.Arm.PIVOT_PORT_1, "drivet");
        kick2 = new TalonFX(Constants.Arm.PIVOT_PORT_2, "drivet");

        foot1 = kick1.getConfigurator();
        foot2 = kick2.getConfigurator();

        encoder = new CANCoder(4, "drivet");
        encoder.configFactoryDefault();
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encoder.configMagnetOffset(Constants.Arm.ENCODER_OFFSET);

        config();
    }

    private static ArmSubsystem singleton;
    public static ArmSubsystem getInstance(){
        if(singleton == null) singleton = new ArmSubsystem();
        return singleton;
    }

    public void config(){
        //Configure the extension motor
        defaultTune = new TalonFXConfiguration();
        defaultTune.Slot0.kS = Constants.Arm.ARM_KF;
        defaultTune.Slot0.kV = 0;
        defaultTune.Slot0.kP = Constants.Arm.ARM_KP;
        defaultTune.Slot0.kI = Constants.Arm.ARM_KI;
        defaultTune.Slot0.kD = Constants.Arm.ARM_KD;

        defaultTune.CurrentLimits.StatorCurrentLimitEnable = true;
        defaultTune.CurrentLimits.StatorCurrentLimit = 0;
        defaultTune.CurrentLimits.SupplyCurrentLimitEnable = true;
        defaultTune.CurrentLimits.SupplyCurrentLimit = 0;

        defaultTune.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        defaultTune.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        defaultTune.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        defaultTune.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Arm.EXTEND_FORWARD_SOFT_LIMIT;
        defaultTune.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        defaultTune.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Arm.EXTEND_REVERSE_SOFT_LIMIT;

        defaultTune.MotionMagic.MotionMagicCruiseVelocity = Constants.Arm.ARM_CRUISE_VELOCITY;
        defaultTune.MotionMagic.MotionMagicAcceleration = Constants.Arm.ARM_ACCELERATION;
        defaultTune.MotionMagic.MotionMagicJerk = 0;

        tuningSlide.apply(tuningConfig);

        posSetter.EnableFOC = true; //Should probably test this.

        //Configure the pivot motors
        defaultFoot = new TalonFXConfiguration();
        defaultFoot.Slot0.kS = Constants.Arm.PIVOT_KF_1;
        defaultFoot.Slot0.kV = 0;
        defaultFoot.Slot0.kP = Constants.Arm.PIVOT_KP_1;
        defaultFoot.Slot0.kI = Constants.Arm.PIVOT_KI_1;
        defaultFoot.Slot0.kD = Constants.Arm.PIVOT_KD_1;

        defaultFoot.CurrentLimits.StatorCurrentLimitEnable = false;
        defaultFoot.CurrentLimits.StatorCurrentLimit = 0;
        defaultFoot.CurrentLimits.SupplyCurrentLimitEnable = false;
        defaultFoot.CurrentLimits.SupplyCurrentLimit = 0;

        defaultFoot.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        defaultFoot.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        defaultFoot.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Arm.PIVOT_FORWARD_SOFT_LIMIT;
        defaultFoot.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        defaultFoot.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Arm.PIVOT_REVERSE_SOFT_LIMIT;

        defaultFoot.MotionMagic.MotionMagicCruiseVelocity = Constants.Arm.PIVOT_CRUISE_VELOCITY;
        defaultFoot.MotionMagic.MotionMagicAcceleration = Constants.Arm.PIVOT_ACCELERATION;
        defaultFoot.MotionMagic.MotionMagicJerk = 0;


        defaultFoot.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;    //The two motors are inverted from each other
        foot1.apply(defaultFoot);
        defaultFoot.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        foot2.apply(defaultFoot);





        // pivot1.configFactoryDefault();
        // pivot1.setNeutralMode(NeutralMode.Brake);
        // pivot1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        // pivot1.config_kF(0, Constants.Arm.PIVOT_KF_1);
        // pivot1.config_kP(0, Constants.Arm.PIVOT_KP_1);
        // pivot1.config_kI(0, Constants.Arm.PIVOT_KI_1);
        // pivot1.config_kD(0, Constants.Arm.PIVOT_KD_1);
        // pivot1.configMotionCruiseVelocity(Constants.Arm.PIVOT_CRUISE_VELOCITY);
        // pivot1.configMotionAcceleration(Constants.Arm.PIVOT_ACCELERATION);

        // pivot2.configFactoryDefault();
        // pivot2.setNeutralMode(NeutralMode.Brake);
        // pivot2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        // pivot2.config_kF(0, Constants.Arm.PIVOT_KF_2);
        // pivot2.config_kP(0, Constants.Arm.PIVOT_KP_2);
        // pivot2.config_kI(0, Constants.Arm.PIVOT_KI_2);
        // pivot2.config_kD(0, Constants.Arm.PIVOT_KD_2);
        // pivot2.configMotionCruiseVelocity(Constants.Arm.PIVOT_CRUISE_VELOCITY);
        // pivot2.configMotionAcceleration(Constants.Arm.PIVOT_ACCELERATION);

        // pivot1.setInverted(InvertType.None);
        // pivot2.setInverted(InvertType.InvertMotorOutput);

        // pivot1.configForwardSoftLimitEnable(true);
        // pivot1.configForwardSoftLimitThreshold(Constants.Arm.PIVOT_FORWARD_SOFT_LIMIT);

        // pivot1.configReverseSoftLimitEnable(true);
        // pivot1.configReverseSoftLimitThreshold(Constants.Arm.PIVOT_REVERSE_SOFT_LIMIT);

        // pivot2.configForwardSoftLimitEnable(true);
        // pivot2.configForwardSoftLimitThreshold(Constants.Arm.PIVOT_FORWARD_SOFT_LIMIT);

        // pivot2.configReverseSoftLimitEnable(true);
        // pivot2.configReverseSoftLimitThreshold(Constants.Arm.PIVOT_REVERSE_SOFT_LIMIT);
    }

    // private PIDController pidController = new PIDController(0, 0, 0);

    // public void setAnglePID(double angle) {
    //     double speed = pidController.calculate(getEncoderAngle(), angle*360/Constants.TAU);
    //     pivot1.set(ControlMode.Velocity, speed);
    //     pivot2.set(ControlMode.Velocity, speed);
    // }

    // public void setTestP(double P) {
    //     pidController.setP(P);
    // }

    // public void setTestI(double I) {
    //     pidController.setP(I);
    // }

    // public void setTestD(double D) {
    //     pidController.setP(D);
    // }

    // public void setPivotP(double P) {
    //     pivot1.config_kP(0, P);
    //     pivot2.config_kP(0, P);
    // }

    // public void setPivotI(double I) {
    //     pivot1.config_kI(0, I);
    //     pivot2.config_kI(0, I);
    // }

    // public void setPivotD(double D) {
    //     pivot1.config_kD(0, D);
    //     pivot2.config_kD(0, D);
    // }

    // public void setExtendP(double P) {
    //     pivot1.config_kP(0, P);
    //     pivot2.config_kP(0, P);
    // }

    // public void setExtendI(double I) {
    //     pivot1.config_kI(0, I);
    //     pivot2.config_kI(0, I);
    // }

    // public void setExtendD(double D) {
    //     pivot1.config_kD(0, D);
    //     pivot2.config_kD(0, D);
    // }

    public void addToAbsoluteOffset(double offset) {
        encoder.configMagnetOffset(encoder.configGetMagnetOffset() + offset);
    }

    // public void zeroPivot1(){
    //     pivot1.setSelectedSensorPosition(0);
        
    // }

    // public void zeroPivot2(){
    //     pivot2.setSelectedSensorPosition(0);
    // }

    // public void zeroPivotSensor(){
    //     zeroPivot1();
    //     zeroPivot2();
    // }

    public void zeroExtend(){   //TODO: High change that this is wrong.
        tromboneSlide.setRotorPosition(0);
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
        return kick1.getSupplyVoltage().getValue();
    }

    public double getPivotSpeed(){
        double pivotSpeed1 = kick1.getVelocity().getValue();
        double pivotSpeed2 = kick2.getVelocity().getValue();

        return (pivotSpeed1+pivotSpeed2)/2;
    }

    //Extends arm to specified position in meters
    public void extendM(double pos){
       extendNU(NRUnits.Extension.mToNU(pos));
    }

    public void extendNU(double nu){
        if(Constants.Logging.ARM) LogManager.appendToLog(nu, "Arm:/Extender/SetNU");
        posSetter.Position = nu;
        tromboneSlide.setControl(new MotionMagicDutyCycle(nu));
    }

    //Basic Percent Output set
    public void set(double speed){
       tromboneSlide.set(speed);
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

    public CANCoderFaults getEncoderFault(){
        CANCoderFaults faults = new CANCoderFaults();
        encoder.getFaults(faults);
        return faults;
    }

    public ErrorCode getLastEncoderError(){
        return encoder.getLastError();
    }

    public boolean encoderOK(){
        CANCoderFaults faults = getEncoderFault();
        if(faults.APIError) return false;
        if(faults.HardwareFault) return false;
        if(faults.MagnetTooWeak) return false;
        if(faults.ResetDuringEn) return false;  //What is this?
        if(faults.UnderVoltage) return false;

        if(getLastEncoderError() != ErrorCode.OK) return false;

        return true;
    }

    public void resetPivotNU(){
        kick1.setRotorPosition(NRUnits.Pivot.degToNU(getEncoderAngle()));
        kick2.setRotorPosition(NRUnits.Pivot.degToNU(getEncoderAngle()));
    }

    //Pivots arm to specified angle (radians) (0 = upright)
    public void pivot(double angle){
        //How does motion magic work with 2 motors?
        double NU = NRUnits.Pivot.radToNU(angle);
        if(Constants.Logging.ARM) LogManager.appendToLog(NU, "Arm:/Pivot2/SetPosition");

        double ff = 0;
        if(NU >= 8552.632){
            ff = 0.00000076 * tromboneSlide.getPosition().getValue()-0.00653;
        }

        ff *= -Math.sin(angle);

        // pivot1.set(ControlMode.MotionMagic, NU);
        // pivot2.set(ControlMode.MotionMagic, NU);
        
        posSetter.Position = NU;
        posSetter.FeedForward = ff;
        kick1.setControl(posSetter);
        kick2.setControl(posSetter);
        // pivot1.set(ControlMode.MotionMagic, NU, DemandType.ArbitraryFeedForward, ff);
        // pivot2.set(ControlMode.MotionMagic, NU, DemandType.ArbitraryFeedForward, ff);
    }

    public void setPivot(double speed){
        kick1.set(speed);
        kick2.set(speed);
    }

    public void holdPivot(){
        double pivotPos = getPivotPos(1);
        posSetter.Position = pivotPos;
        kick1.setControl(posSetter);
        kick2.setControl(posSetter);
        // pivot1.set(ControlMode.MotionMagic, pivotPos1);

        // double pivotPos2 = getPivotPos(2);
        // pivot2.set(ControlMode.MotionMagic, pivotPos2);
    }

    public double getExtendNUSpeed(){
        return tromboneSlide.getVelocity().getValue();
    }

    public void holdArm(){
        posSetter.Position = getExtendNU();
        tromboneSlide.setControl(posSetter);
    }

    //Returns the angle of the arm
    public double getAngle(){
        return (getPivotAngle(1) + getPivotAngle(2))/2;
    }

    public double getExtendNU(){
        return tromboneSlide.getPosition().getValue();
    }

    //Returns the extension of the arm in meters
    public double getLength(){
        double pos = getExtendNU();

        return NRUnits.Extension.NUToM(pos);
    }

    public double getArmStatorCurrent() {
        return tromboneSlide.getStatorCurrent().getValue();
    }
    
    public double getArmSupplyCurrent() {
        return tromboneSlide.getSupplyCurrent().getValue();
    }

    public double getPivotStatorCurrent() {
        return kick1.getStatorCurrent().getValue();
    }

    public double getPivotSupplyCurrent() {
        return kick1.getSupplyCurrent().getValue();
    }

    // public void setBrakeMode(NeutralMode n){
    //     pivot1.setNeutralMode(n);
    //     pivot2.setNeutralMode(n);
    // }

    //This is TEMPORARY
    public double getPivotPos(int n){
        if(n == 1){
            return kick1.getPosition().getValue();
        }
        else if (n==2){
            return kick2.getPosition().getValue();
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
        return kick1.getStatorCurrent().getValue();
    }

    public double getStatorCurrent2(){
        return kick2.getStatorCurrent().getValue();
    }

    public double getPivotStator(){
        return (getStatorCurrent1()+getStatorCurrent2())/2;
    }

    public double getSupplyCurrent1(){
        return kick1.getSupplyCurrent().getValue();
    }

    public double getSupplyCurrent2(){
        return kick2.getSupplyCurrent().getValue();
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
        tromboneSlide.configMotionCruiseVelocity(72_000);

        pivot1.configMotionCruiseVelocity(Constants.Arm.ARM_CRUISE_VELOCITY);
        pivot2.configMotionCruiseVelocity(Constants.Arm.ARM_CRUISE_VELOCITY);
    }

    public void setDefaultAcceleration() {
        tromboneSlide.configMotionAcceleration(45_000);

        pivot1.configMotionAcceleration(Constants.Arm.ARM_ACCELERATION);
        pivot2.configMotionAcceleration(Constants.Arm.ARM_ACCELERATION);
    }

    public boolean armAtZero(){
        return Math.abs(getEncoderAngle()) < 1; //degree
    }

    @Override
    public void periodic() {
        if(Constants.Logging.ARM) {
            //Extender
            LogManager.appendToLog(NRUnits.Pivot.NUToRad(tromboneSlide.getSelectedSensorPosition()), "Arm:/Extender/Position");
            LogManager.appendToLog(tromboneSlide.getStatorCurrent(), "Arm:/Extender/Stator");
            LogManager.appendToLog(tromboneSlide.getSupplyCurrent(), "Arm:/Extender/Supply");
            
            
            //Pivot1
            LogManager.appendToLog(NRUnits.Pivot.NUToRad(ArmSubsystem.getInstance().getAngle()), "Arm:/RelativeAngle");
            LogManager.appendToLog(NRUnits.Pivot.NUToRad(getEncoderAngle()), "Arm:/Pivot1/AbsolutePosition");
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