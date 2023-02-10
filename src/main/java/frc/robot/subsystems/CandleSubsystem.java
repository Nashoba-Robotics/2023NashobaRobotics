package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import frc.robot.Constants;

public class CandleSubsystem {
    
    private final static int LED_COUNT = 0;

    CANdle candle;

    public CandleSubsystem() {
        candle = new CANdle(Constants.Misc.CANDLE_PORT);
    }

    private static CandleSubsystem instance;
    public static CandleSubsystem getInstance() {
        if(instance == null) instance = new CandleSubsystem();
        return instance;
    }

    public enum CandleState {

    }

    public void set(CandleState state) {
        switch(state) {

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
