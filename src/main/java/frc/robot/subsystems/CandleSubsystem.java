package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.NewTabs;

public class CandleSubsystem extends SubsystemBase {
    
    private final static int LED_COUNT = 116;
    // private static final int[] yellow = {0,0,0};
    private static final int[] yellow = {255,80,0};
    private final static int[] purple = {148,0,211};
    private final static int[] defaultColor = {0xFF, 0x10, 0x0};
    // private final static int[] defaultColor = {0xFF, 0x60, 0x03};

    CANdle candle;

    public CandleSubsystem() {
        candle = new CANdle(Constants.Misc.CANDLE_PORT, "drivet");
    }

    private static CandleSubsystem instance;
    public static CandleSubsystem getInstance() {
        if(instance == null) instance = new CandleSubsystem();
        return instance;
    }

    public enum CandleState {
        NONE,
        ENABLED,
        DISABLED,
        AUTO,
        WANT_CONE,
        WANT_CUBE,
        HAVE_CONE,
        HAVE_CUBE,
        DOUBLE_STATION,
        BAD,
        FUN,
        SYSTEM_CHECK,
        SYSTEM_GOOD,
        SYSTEM_BAD,
        PARTIAL_CHECK_1,
        FORDIANI
    }

    private Animation disabledAnimation;
    @Override
    public void periodic() {
        if(DriverStation.isDisabled()){
            if(!ArmSubsystem.getInstance().encoderOK()){
                disabledAnimation = new StrobeAnimation(255, 0, 0, 0, 0.3, LED_COUNT);
                NewTabs.putDouble("Errors", "EncoderState", -1);
            }
            else if(!SwerveDriveSubsystem.getInstance().encodersOK()){
                disabledAnimation = new StrobeAnimation(255, 80, 0, 0, 0.3, LED_COUNT);
                NewTabs.putDouble("Errors", "EncoderState", -2);
            }
            else{
                disabledAnimation = new LarsonAnimation(defaultColor[0], defaultColor[1], defaultColor[2], 0, 0.3, LED_COUNT, BounceMode.Back, 7);
                NewTabs.putDouble("Errors", "EncoderState", 1);
            }
            candle.animate(disabledAnimation);
            /*
             * Can add pre-match diagnostic stuff in here
             * 1. Check the arm encoder
             * 2. Check the drive encoders
             * 3. Check the arm angle
             */
        }
    }

    public void set(CandleState state) {
        candle.clearAnimation(0);
        switch(state) {
            case NONE:
                candle.clearAnimation(0);
                break;
            case ENABLED:
                candle.clearAnimation(0);
                //candle.animate(new StrobeAnimation(0, 255, 0, 0, 1, LED_COUNT));
                // candle.setLEDs(0, 255, 0);
                candle.setLEDs(255, 255, 255);

                break;
            case DISABLED:
                candle.clearAnimation(0);
                // candle.animate(new RgbFadeAnimation(0.5, 0.2, LED_COUNT, 0));
                candle.animate(new LarsonAnimation(defaultColor[0], defaultColor[1], defaultColor[2], 0, 0.3, LED_COUNT, BounceMode.Back, 7));
                // candle.setLEDs(defaultColor[0], defaultColor[1], defaultColor[2]);
                break;
            case AUTO:
                candle.animate(new RainbowAnimation(1, 0.8, LED_COUNT));
                break;
            case WANT_CONE:
                candle.clearAnimation(0);
                candle.setLEDs(yellow[0], yellow[1], yellow[2]);
                break;
            case WANT_CUBE:
                candle.clearAnimation(0);
                candle.setLEDs(purple[0], purple[1], purple[2]);
                break;
            case HAVE_CONE:
                //candle.animate(new StrobeAnimation(yellow[0], yellow[1], yellow[2], 0, 0.01, LED_COUNT));
                candle.clearAnimation(0);
                candle.setLEDs(0, 230, 0);
                break;
            case HAVE_CUBE:
                candle.animate(new StrobeAnimation(purple[0], purple[1], purple[2], 0, 0.01, LED_COUNT));
                break;
            case DOUBLE_STATION:
                candle.clearAnimation(0);
                candle.setLEDs(0, 0, 255);
                break;
            case BAD:
                candle.animate(new StrobeAnimation(255, 0, 0, 0, 0.01, LED_COUNT));
                break;
            case FUN:
                Animation a = new RgbFadeAnimation(0.7, 0.7, LED_COUNT);
                candle.clearAnimation(0);
                candle.animate(a);
                break;
            case SYSTEM_CHECK:
                candle.clearAnimation(0);
                candle.animate(new StrobeAnimation(255, 255, 255, 0, 0.01, LED_COUNT));
                break;
            case SYSTEM_GOOD:
                candle.clearAnimation(0);
                candle.setLEDs(0, 255, 0);
                break;
            case SYSTEM_BAD:
                candle.clearAnimation(0);
                candle.animate(new StrobeAnimation(255, 0, 0, 0, 0.03, LED_COUNT));
                break;
            case PARTIAL_CHECK_1:
                candle.clearAnimation(0);
                candle.animate(new StrobeAnimation(yellow[0], yellow[1], yellow[2], 0, 0.01, LED_COUNT));
                break;
            case FORDIANI:
                candle.clearAnimation(0);
                candle.setLEDs(255, 0, 0);
                break;
        }

    }

    public void set(int[] colors) {
        int animSlot = 0;
        candle.clearAnimation(animSlot);
        candle.setLEDs(colors[0], colors[1], colors[2]);
    }

    public void set(int[] colors, Animation animation) {
        candle.animate(animation);
        candle.setLEDs(colors[0], colors[1], colors[2]);
    }

}
