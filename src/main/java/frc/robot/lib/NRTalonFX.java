package frc.robot.lib;

import javax.management.ConstructorParameters;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.ControlModeValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

public class NRTalonFX {
    private TalonFX motor;
    private TalonFXConfigurator configurator;
    private TalonFXConfiguration config;

    private boolean foc = true;
    private boolean overrideBrake = false;

    private DutyCycleOut percentOut = new DutyCycleOut(0, foc, overrideBrake);
    private VelocityDutyCycle velocity = new VelocityDutyCycle(0, foc, 0, 0, overrideBrake);
    private MotionMagicDutyCycle motionMagic = new MotionMagicDutyCycle(0, foc, 0, 0, overrideBrake);

    public NRTalonFX(int id){
        motor = new TalonFX(id);
        configurator = motor.getConfigurator();
        config = new TalonFXConfiguration();
    }
    public NRTalonFX(int id, String CANivore){
        motor = new TalonFX(id, CANivore);
        configurator = motor.getConfigurator();
        config = new TalonFXConfiguration();
    }

    public void configFactoryDefault(){
        config = new TalonFXConfiguration();
        configurator.apply(config);
    }

    // I don't think we're ever going to change slots for PID. If we ever do, I'll update this
    public void setKS(double kS){
        config.Slot0.kS = kS;
        //Do I need to apply this to the configurator or not?
    }
    public void setKV(double kV){
        config.Slot0.kV = kV;
    }
    public void setKP(double kP){
        config.Slot0.kP = kP;
    }
    public void setKI(double kI){
        config.Slot0.kI = kI;
    }
    public void setKD(double kD){
        config.Slot0.kD = kD;
    }

    public void configPID(double kP, double kI, double kD){
        setKP(kP);
        setKI(kI);
        setKD(kD);
    }
    public void configPID(double kS, double kV, double kP, double kI, double kD){
        setKS(kS);
        setKV(kV);
        configPID(kP, kI, kD);
    }

    public void setNeutralMode(NeutralModeValue neutralMode){
        config.MotorOutput.NeutralMode = neutralMode;
    }
    public void setInverted(InvertedValue invert){
        config.MotorOutput.Inverted = invert;
    }

    public void setCruiseVelocity(double cruiseVelocity){
        config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
    }
    public void setAcceleration(double acceleration){
        config.MotionMagic.MotionMagicAcceleration = acceleration;
    }
    public void setJerk(double jerk){
        config.MotionMagic.MotionMagicJerk = jerk;
    }

    public void enableStatorCurrentLimit(boolean enable){
        config.CurrentLimits.StatorCurrentLimitEnable = enable;
    }
    public void setStatorCurrentLimit(double limit){
        enableStatorCurrentLimit(true);
        config.CurrentLimits.StatorCurrentLimit = limit;
    }
    public void enableSupplyCurrentLimit(boolean enable){
        config.CurrentLimits.SupplyCurrentLimitEnable = enable;
    }
    public void setSupplyCurrentLimit(double limit){
        enableSupplyCurrentLimit(true);
        config.CurrentLimits.StatorCurrentLimit = limit;
    }

    public void enableForwardSoftLimit(boolean enable){
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    }
    public void setForwardSoftLimit(double limit){
        enableForwardSoftLimit(true);
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = limit;
    }
    public void enableReverseSoftLimit(boolean enable){
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    }
    public void setReverseSoftLimit(double limit){
        enableReverseSoftLimit(true);
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = limit;
    }

    public void setSoftLimits(double forwardLimit, double reverseLimit){
        setForwardSoftLimit(forwardLimit);
        setReverseSoftLimit(reverseLimit);
    }
    
    public void enableFOC(boolean enable){
        foc = enable;
    }
    public void enableBrakeOverride(boolean enable){
        overrideBrake = enable;
    }

    public void set(double speed){
        motor.set(speed);
    }

    public void set(ControlMode controlMode, double value){
        switch(controlMode){
            case PercentOutput:
                percentOut.Output = value;
                percentOut.EnableFOC = foc;
                percentOut.OverrideBrakeDurNeutral = overrideBrake;
                motor.setControl(percentOut);
                break;
            case Velocity:
                velocity.Velocity = value;
                velocity.EnableFOC = foc;
                velocity.OverrideBrakeDurNeutral = overrideBrake;
                motor.setControl(velocity);
                break;
            case MotionMagic:
                motionMagic.Position = value;
                motionMagic.EnableFOC = foc;
                motionMagic.OverrideBrakeDurNeutral = overrideBrake;
                motor.setControl(motionMagic);
                break;
            default:
                motor.set(0);
                break;
        }
    }

    public void set(ControlMode controlMode, double value, double aff){
        switch(controlMode){
            case PercentOutput:
                percentOut.Output = value;
                percentOut.EnableFOC = foc;
                percentOut.OverrideBrakeDurNeutral = overrideBrake;
                motor.setControl(percentOut);
                break;
            case Velocity:
                velocity.Velocity = value;
                velocity.EnableFOC = foc;
                velocity.OverrideBrakeDurNeutral = overrideBrake;
                velocity.FeedForward = aff;
                motor.setControl(velocity);
                break;
            case MotionMagic:
                motionMagic.Position = value;
                motionMagic.EnableFOC = foc;
                motionMagic.OverrideBrakeDurNeutral = overrideBrake;
                motionMagic.FeedForward = aff;
                motor.setControl(motionMagic);
                break;
            default:
                motor.set(0);
                break;
        }
    }

    public double getVelocity(){
        return motor.getVelocity().getValue();
    }
    public double getNU(){
        return motor.getPosition().getValue();
    }
    public double getStator(){
        return motor.getStatorCurrent().getValue();
    }
    public double getSupplyCurrent(){
        return motor.getSupplyCurrent().getValue();
    }
    public double getTorqueCurrent(){
        return motor.getTorqueCurrent().getValue();
    }

    public double getSupplyVoltage(){
        return motor.getSupplyVoltage().getValue();
    }
    public String getCANBus(){
        return motor.getCANBus();
    }

    //There is a 20% chance that this works first try
    public boolean motorOK(){
        if(motor.getFault_BootDuringEnable().getValue()) return false;
        if(motor.getFault_DeviceTemp().getValue()) return false;
        if(motor.getFault_FusedSensorOutOfSync().getValue()) return false;
        if(motor.getFault_Hardware().getValue()) return false;
        if(motor.getFault_MissingRemoteSensor().getValue()) return false;
        if(motor.getFault_OverSupplyV().getValue()) return false;
        if(motor.getFault_ProcTemp().getValue()) return false;
        if(motor.getFault_Undervoltage().getValue()) return false;
        if(motor.getFault_UnstableSupplyV().getValue()) return false;

        if(motor.getStickyFault_BootDuringEnable().getValue()) return false;
        if(motor.getStickyFault_DeviceTemp().getValue()) return false;
        if(motor.getStickyFault_FusedSensorOutOfSync().getValue()) return false;
        if(motor.getStickyFault_Hardware().getValue()) return false;
        if(motor.getStickyFault_MissingRemoteSensor().getValue()) return false;
        if(motor.getStickyFault_OverSupplyV().getValue()) return false;
        if(motor.getStickyFault_ProcTemp().getValue()) return false;
        if(motor.getStickyFault_Undervoltage().getValue()) return false;
        if(motor.getStickyFault_UnstableSupplyV().getValue()) return false;

        return true;
    }
}
