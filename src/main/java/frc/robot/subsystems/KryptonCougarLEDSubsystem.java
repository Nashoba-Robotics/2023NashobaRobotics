package frc.robot.subsystems;

import java.io.Console;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem.ArmStatus;


/*
 * Works fairly well.
 * 
 * For in-game diagnostic, the colors should be in more obvious positions
 * TODO: Figure out a way to make it so when something breaks, it is obvious, but we can still play
 *      When setting robotState, we shuold be able to take a portion out of it and tell the robot that it is dying
 */
public class KryptonCougarLEDSubsystem extends SubsystemBase{
    private CANdle candle;
    private CANdleConfiguration candleConfig;

    private final double blinkSpeed = 0.01;
    private final double rainbowSpeed = 0.8;
    private final double flowSpeed = 0.01;
    private final double fadeSpeed = 0.5;
    private final double bounceSpeed = 0.3;

    public static LightState state = LightState.DEFAULT;

    private final Color cone = new Color(255, 80, 0);
    private final Color haveCone = new Color(0, 255, 0);
    private final Color cube = new Color(148, 0, 211);
    private final Color singleStation = new Color(0, 0, 255);
    private final Color disabled = new Color(0xFF, 0x10, 0x0);
    private final Color defaultColor = new Color(255, 255, 255);
    private final Color good = new Color(0, 255, 0);
    private final Color bad = new Color(255, 0, 0);

    private Light armState = new Light(0, 2);
    private Light swerveState = new Light(4, 4);
    private Light fmsState = new Light(3, 1);
    private Light robotState = new Light(8, 108);



    @Override
    public void periodic() {
        if(DriverStation.isDisabled()){
            if(ArmSubsystem.status == ArmStatus.OK) armState.solid(good);
            else armState.solid(bad);
            if(DriverStation.isFMSAttached()) fmsState.solid(good);
            else fmsState.solid(cone);
            
            robotState.flow(disabled);
        }
        else if(DriverStation.isAutonomous()){
            robotState.rainbow();            
        }
        else if(DriverStation.isTeleopEnabled()){
            switch(state){
                case DEFAULT:
                    robotState.solid(defaultColor);
                    break;
                case WANT_CONE:
                    robotState.solid(cone);
                    break;
                case HAVE_CONE:
                    robotState.solid(haveCone);
                    break;
                case WANT_CUBE:
                    robotState.solid(cube);
                    break;
                case HAVE_CUBE:
                    break;
                case SINGLE_STATION:
                    robotState.solid(singleStation);
                    break;
                default:
                    robotState.solid(defaultColor);
                    break;
            }
        }
    }

    public KryptonCougarLEDSubsystem(){
        candle = new CANdle(Constants.Misc.CANDLE_PORT, "drivet");
        candle.clearAnimation(0);

        candleConfig = new CANdleConfiguration();

        candleConfig.enableOptimizations = true;
        candleConfig.disableWhenLOS = false;
        candleConfig.statusLedOffWhenActive = true;
        candleConfig.stripType = LEDStripType.RGB;
        candleConfig.brightnessScalar = 0.5;

        candle.configAllSettings(candleConfig);
    }

    public enum LightState{
        DEFAULT,
        WANT_CONE,
        HAVE_CONE,
        WANT_CUBE,
        HAVE_CUBE,
        SINGLE_STATION
    }

    private class Color{
        int r;
        int g;
        int b;

        public Color(int r, int g, int b){
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    private class Light{
        int start;
        int length;
        // Color color;
        Direction flowDirection = Direction.Forward;

        public Light(int start, int length){
            this.start = start;
            this.length = length;
        }

        public void setFlowDirection(Direction d){
            flowDirection = d;
        }

        public void solid(Color color){
            candle.setLEDs(color.r, color.g, color.b, 0, this.start, this.length);
        }
        public void blink(Color color){
            candle.animate(new StrobeAnimation(color.r, color.g, color.b, this.start, blinkSpeed, this.length));
        }
        public void flow(Color color){
            candle.animate(new ColorFlowAnimation(color.r, color.g, color.b, 0, flowSpeed, length, flowDirection, start));
        }
        public void bounce(Color color){
            candle.animate(new LarsonAnimation(color.r, color.g, color.b, 0, bounceSpeed, length, BounceMode.Back, 7));
        }
        public void fade(Color color){
            candle.animate(new SingleFadeAnimation(color.r, color.g, color.b, 0, fadeSpeed, length, start));
        }
        public void rainbow(){
            candle.animate(new RainbowAnimation(0, rainbowSpeed, length, false, start));
        }
    }
}
