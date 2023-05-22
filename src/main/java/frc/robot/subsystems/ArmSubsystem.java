package frc.robot.subsystems;

import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.CANcoderConfigurator;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.MagnetHealthValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LogManager;
import frc.robot.Tabs;
import frc.robot.lib.math.NRUnits;


//TODO: Change all of the diagnostic functions to read only the first pivot motor
public class ArmSubsystem extends SubsystemBase {
    private TalonFX tromboneSlide;  //Controls the extension/retraction of the arm
    private TalonFXConfigurator tuningSlide;
    private TalonFXConfiguration tuneConfig;

    private CANcoder encoder;
    private CANcoderConfigurator encoderConfigurator;
    private CANcoderConfiguration encoderConfig;

    private MotionMagicDutyCycle extendSetter;
    private MotionMagicDutyCycle pivotSetter;

    

    private TalonFX kick1, kick2; //Control the pivoting of the entire arm
    private TalonFXConfigurator foot;
    private TalonFXConfiguration footConfig;

    public ArmSubsystem(){
        tromboneSlide = new TalonFX(Constants.Arm.ARM_PORT, "drivet");
        tuningSlide = tromboneSlide.getConfigurator();
        extendSetter = new MotionMagicDutyCycle(0, true, 0, 0, false); 
        pivotSetter = new MotionMagicDutyCycle(0, true, 0, 0, false);


        kick1 = new TalonFX(Constants.Arm.PIVOT_PORT_1, "drivet");
        kick2 = new TalonFX(Constants.Arm.PIVOT_PORT_2, "drivet");
        Follower kickFollow = new Follower(Constants.Arm.PIVOT_PORT_1, true);
        kick2.setControl(kickFollow);

        foot = kick1.getConfigurator();

        encoder = new CANcoder(4, "drivet");
        encoderConfigurator = encoder.getConfigurator();
        // encoder.configFactoryDefault();
        // encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        // encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        // encoder.configMagnetOffset(Constants.Arm.ENCODER_OFFSET);

        config();
    }

    private static ArmSubsystem singleton;
    public static ArmSubsystem getInstance(){
        if(singleton == null) singleton = new ArmSubsystem();
        return singleton;
    }

    public void config(){
        kick2.setControl(new Follower(Constants.Arm.PIVOT_PORT_1, true));

        //Configure the extension motor
        tuneConfig = new TalonFXConfiguration();
        tuneConfig.Slot0.kS = 0;
        tuneConfig.Slot0.kV = Constants.Arm.ARM_KF;
        tuneConfig.Slot0.kP = Constants.Arm.ARM_KP;
        tuneConfig.Slot0.kI = Constants.Arm.ARM_KI;
        tuneConfig.Slot0.kD = Constants.Arm.ARM_KD;

        tuneConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        tuneConfig.CurrentLimits.StatorCurrentLimit = 0;
        tuneConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        tuneConfig.CurrentLimits.SupplyCurrentLimit = 0;

        tuneConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        tuneConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        tuneConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        tuneConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Arm.EXTEND_FORWARD_SOFT_LIMIT;
        tuneConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        tuneConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Arm.EXTEND_REVERSE_SOFT_LIMIT;

        tuneConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Arm.ARM_CRUISE_VELOCITY;
        tuneConfig.MotionMagic.MotionMagicAcceleration = Constants.Arm.ARM_ACCELERATION;
        tuneConfig.MotionMagic.MotionMagicJerk = 0;

        tuningSlide.apply(tuneConfig);

        // posSetter.EnableFOC = true; //Should probably test this.

        //Configure the pivot motors
        footConfig = new TalonFXConfiguration();
        footConfig.Slot0.kS = 0;
        footConfig.Slot0.kV = Constants.Arm.PIVOT_KF_1;
        footConfig.Slot0.kP = Constants.Arm.PIVOT_KP_1;
        footConfig.Slot0.kI = Constants.Arm.PIVOT_KI_1;
        footConfig.Slot0.kD = Constants.Arm.PIVOT_KD_1;

        footConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        footConfig.CurrentLimits.StatorCurrentLimit = 0;
        footConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        footConfig.CurrentLimits.SupplyCurrentLimit = 0;

        footConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        footConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        footConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Arm.PIVOT_FORWARD_SOFT_LIMIT;
        footConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        footConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Arm.PIVOT_REVERSE_SOFT_LIMIT;

        footConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Arm.PIVOT_CRUISE_VELOCITY;
        footConfig.MotionMagic.MotionMagicAcceleration = Constants.Arm.PIVOT_ACCELERATION;
        footConfig.MotionMagic.MotionMagicJerk = 0;

        foot.apply(footConfig);


        encoderConfig = new CANcoderConfiguration();
        
        encoderConfig.FutureProofConfigs = true;

        encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        encoderConfig.MagnetSensor.MagnetOffset = Constants.Arm.ENCODER_OFFSET;

        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;   //TODO: Make sure this is going in the correct direction

        encoderConfigurator.apply(encoderConfig);
    }

    public void addToAbsoluteOffset(double offset) {
        encoderConfig.MagnetSensor.MagnetOffset += offset;
        encoderConfigurator.apply(encoderConfig);
    }

    public void zeroExtend(){
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
        return kick1.getVelocity().getValue();
    }

    //Extends arm to specified position in meters
    public void extendM(double pos){
       extendNU(NRUnits.Extension.mToNU(pos));
    }

    public void extendNU(double nu){
        if(Constants.Logging.ARM) LogManager.appendToLog(nu, "Arm:/Extender/SetNU");
        extendSetter.Position = nu;
        tromboneSlide.setControl(new MotionMagicDutyCycle(nu));
    }

    //Basic Percent Output set
    public void set(double speed){
       tromboneSlide.set(speed);
    }

    public double getEncoderDeg(){
        return encoder.getAbsolutePosition().getValue() * 360;
    }

    public boolean encoderOK(){
        if(encoder.getFault_BadMagnet().getValue()) return false;
        if(encoder.getFault_BootDuringEnable().getValue()) return false;
        if(encoder.getFault_Hardware().getValue()) return false;
        if(encoder.getFault_Undervoltage().getValue()) return false;

        if(encoder.getFaultField().getError() != StatusCode.OK) return false;

        if(encoder.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Green) return false;

        return true;
    }

    public void resetPivotNU(){
        kick1.setRotorPosition(NRUnits.Pivot.degToRot(getEncoderDeg()));
    }

    //Pivots arm to specified angle (radians) (0 = upright)
    public void pivot(double angle){
        double NU = NRUnits.Pivot.radToRot(angle);
        if(Constants.Logging.ARM) LogManager.appendToLog(NU, "Arm:/Pivot2/SetPosition");

        // double ff = 0;
        // if(NU >= 8552.632){
        //     ff = 0.00000076 * tromboneSlide.getPosition().getValue()-0.00653;
        // }

        // ff *= -Math.sin(angle);

  
        pivotSetter.Slot = 0;
        pivotSetter.Position = NU;
        // posSetter.FeedForward = ff;
        kick1.setControl(pivotSetter);
    }

    public void setPivot(double speed){
        kick1.set(speed);
    }

    public void holdPivot(){
        double pivotPos = getPivotPos();
        pivotSetter.Position = pivotPos;
        kick1.setControl(pivotSetter);
    }

    public double getExtendVelocity(){
        return tromboneSlide.getVelocity().getValue();
    }

    public void holdArm(){
        extendSetter.Position = getExtendNU();
        tromboneSlide.setControl(extendSetter);
    }

    //Returns the angle of the arm
    public double getAngle(){
        return getPivotDeg() * Constants.TAU/360;
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

    public void setPivotBrakeMode(NeutralModeValue n){
        footConfig.MotorOutput.NeutralMode = n;

        foot.apply(footConfig);
    }

    public void setExtenBrakeMode(NeutralModeValue n){
        tuneConfig.MotorOutput.NeutralMode = n;

        tuningSlide.apply(tuneConfig);
    }

    //This is TEMPORARY
    public double getPivotPos(){
        return kick1.getPosition().getValue();
    }

    public double getPivotDeg(){
        return NRUnits.Pivot.rotToRad(getPivotPos()) * 360/Constants.TAU;
    }

    public double getStatorCurrent1(){
        return kick1.getStatorCurrent().getValue();
    }

    public double getStatorCurrent2(){
        return kick2.getStatorCurrent().getValue();
    }

    public double getPivotStator(){
        return getStatorCurrent1();
    }

    public double getSupplyCurrent1(){
        return kick1.getSupplyCurrent().getValue();
    }

    public double getSupplyCurrent2(){
        return kick2.getSupplyCurrent().getValue();
    }

    public double getPivotSupply(){
        return getSupplyCurrent1();
    }

    public void setPivotCruiseVelocity(double cruiseVelocity) {
        footConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
        foot.apply(footConfig);
    }

    public void setPivotAcceleration(double acceleration) {
        footConfig.MotionMagic.MotionMagicAcceleration = acceleration;
        foot.apply(footConfig);
    }   

    public void setPivotJerk(double jerk){
        footConfig.MotionMagic.MotionMagicJerk = jerk;
        foot.apply(footConfig);
    }

    public void setExtendCruiseVelocity(double cruiseVelocity) {
        tuneConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
        tuningSlide.apply(tuneConfig);
    }
    public void setExtendAcceleration(double acceleration) {
        tuneConfig.MotionMagic.MotionMagicAcceleration = acceleration;
        tuningSlide.apply(tuneConfig);
    }
    public void setExtendJerk(double jerk){
        tuneConfig.MotionMagic.MotionMagicJerk = jerk;
        tuningSlide.apply(tuneConfig);
    }

    public void setMotionMagicConfigs(double extendCVel, double extendAcc, double pivotCVel, double pivotAcc){
        setExtendCruiseVelocity(extendCVel);
        setExtendAcceleration(extendAcc);

        setPivotCruiseVelocity(pivotCVel);
        setPivotAcceleration(pivotAcc);
    }

    public void setDefaultCruiseVelocity() {
        setExtendCruiseVelocity(Constants.Arm.ARM_CRUISE_VELOCITY);
        setPivotCruiseVelocity(Constants.Arm.PIVOT_CRUISE_VELOCITY);
    }

    public void setDefaultAcceleration() {
        setExtendAcceleration(Constants.Arm.ARM_ACCELERATION);
        setPivotAcceleration(Constants.Arm.PIVOT_ACCELERATION);
    }

    public boolean armAtZero(){
        return Math.abs(getEncoderDeg()) < 1; //degree
    }

    @Override
    public void periodic() {
        if(Constants.Logging.ARM) {
            //Extender
            LogManager.appendToLog(getExtendNU(), "Arm:/Extender/Position");
            LogManager.appendToLog(tromboneSlide.getStatorCurrent(), "Arm:/Extender/Stator");
            LogManager.appendToLog(tromboneSlide.getSupplyCurrent(), "Arm:/Extender/Supply");
            
            
            //Pivot1
            LogManager.appendToLog(NRUnits.Pivot.rotToRad(ArmSubsystem.getInstance().getAngle()), "Arm:/RelativeAngle");
            LogManager.appendToLog(NRUnits.Pivot.rotToRad(getEncoderDeg()), "Arm:/Pivot1/AbsolutePosition");
            LogManager.appendToLog(getStatorCurrent1(), "Arm:/Pivot1/Stator");
            LogManager.appendToLog(getSupplyCurrent1(), "Arm:/Pivot1/Supply");

            //Pivot2
            LogManager.appendToLog(getPivotPos(), "Arm:/Pivot2/Position");
            LogManager.appendToLog(getStatorCurrent2(), "Arm:/Pivot2/Stator");
            LogManager.appendToLog(getSupplyCurrent2(), "Arm:/Pivot2/Supply");
            
        }

        // SmartDashboard.putNumber("PivotCurrent", pivot1.getStatorCurrent());

        Tabs.Comp.displayPivotAngle(getAngle());
        Tabs.Comp.displayEncoderAngle(getEncoderDeg());
        Tabs.Comp.displayExtendNU(getExtendNU());
    }
}