package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
        HAVE_CUBE
    }

    public void set(CandleState state) {
        switch(state) {
            case NONE:
                candle.clearAnimation(0);
                break;
            case ENABLED:
                candle.clearAnimation(0);
                //candle.animate(new StrobeAnimation(0, 255, 0, 0, 1, LED_COUNT));
                candle.setLEDs(0, 255, 0);
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
                candle.animate(new StrobeAnimation(yellow[0], yellow[1], yellow[2], 0, 0.01, LED_COUNT));
                break;
            case HAVE_CUBE:
                candle.animate(new StrobeAnimation(purple[0], purple[1], purple[2], 0, 0.01, LED_COUNT));
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
