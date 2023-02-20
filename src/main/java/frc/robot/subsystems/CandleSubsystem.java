package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import frc.robot.Constants;

public class CandleSubsystem {
    
    private final static int LED_COUNT = 39;
    private static final int[] yellow = {255,255,51};
    private final static int[] purple = {148,0,211};
    private final static int[] amber = {0xFF, 0xBF, 0};

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
                candle.animate(new StrobeAnimation(0xFF, 0xBF, 0, 0, 0.5, LED_COUNT));
                break;
            case DISABLED:
                candle.setLEDs(0xFF, 0xBF, 0);
                break;
            case AUTO:
                candle.animate(new RainbowAnimation());
                break;
            case WANT_CONE:
                candle.setLEDs(yellow[0], yellow[1], yellow[2]);
                break;
            case WANT_CUBE:
                candle.setLEDs(purple[0], purple[1], purple[2]);
                break;
            case HAVE_CONE:
                candle.animate(new StrobeAnimation(yellow[0], yellow[1], yellow[2], 0, 0.5, LED_COUNT));
                break;
            case HAVE_CUBE:
                candle.animate(new StrobeAnimation(purple[0], purple[1], purple[2], 0, 0.5, LED_COUNT));
                break;
        }

    }

    public void set(int[] colors) {
        candle.clearAnimation(0);
        candle.setLEDs(colors[0], colors[1], colors[2]);
    }

    public void set(int[] colors, Animation animation) {
        candle.animate(animation);
        candle.setLEDs(colors[0], colors[1], colors[2]);
    }

}
