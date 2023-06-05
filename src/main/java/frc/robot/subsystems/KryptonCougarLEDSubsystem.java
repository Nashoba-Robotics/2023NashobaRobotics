package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KryptonCougarLEDSubsystem extends SubsystemBase{
    private CANdle candle;
    private CANdleConfiguration candleConfig;

    @Override
    public void periodic() {
        candle.setLEDs(defaultColor.r, defaultColor.g, defaultColor.b, 0, robotState.start, robotState.length);
    }

    public KryptonCougarLEDSubsystem(){
        candle = new CANdle(0, "drivet");
        candleConfig = new CANdleConfiguration();

        candleConfig.enableOptimizations = true;
        candleConfig.stripType = LEDStripType.RGB;
        candleConfig.brightnessScalar = 1;

        candle.configAllSettings(candleConfig);
    }

    Color cone = new Color(0, 0, 0);
    Color cube = new Color(0, 0, 0);
    Color disabled = new Color(0, 0, 0);
    Color defaultColor = new Color(0, 0, 0);
    Color bad = new Color(0, 0, 0);

    Light armState = new Light(0, 2);
    Light swerveState = new Light(2, 2);
    Light robotState = new Light(8, 152);

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

        public Light(int start, int length){
            this.start = start;
            this.length = length;
        }
    }
}
