package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SmartLEDSubsystem extends SubsystemBase{
    private static CANdle candle;

    private static final Color white = new Color(255, 255, 255);
    private static final Color black = new Color(0, 0, 0);
    private static final Color yellow = new Color(255, 80, 0);
    private static final Color purple = new Color(148, 0, 211);
    private static final Color green = new Color(0, 255, 0);
    private static final Color red = new Color(255, 0, 0);
    private static final Color orange = new Color(0xFF, 0x10, 0x0);
    private static final Color blue = new Color(0, 0, 255);

    public SmartLEDSubsystem(){
        candle = new CANdle(Constants.Misc.CANDLE_PORT, "drivet");
    }
    private static boolean full = true;
    private static Color statusColor;
    private static Color stateColor;
    @Override
    public void periodic() {
        //Front facing away from DS
        if(!full){
            LEDSegment.MainStrip.clearAnimation();
            if(SwerveDriveSubsystem.getInstance().getGyroAngle() >= -Constants.TAU/8
            && SwerveDriveSubsystem.getInstance().getGyroAngle() <= Constants.TAU/8){
                LEDSegment.FLStrip.setColor(stateColor);
                LEDSegment.FRStrip.setColor(stateColor);
                if(DriverStation.getAlliance() == Alliance.Blue){
                    LEDSegment.BLStrip.setColor(stateColor);
                    LEDSegment.BRStrip.setColor(statusColor);
                }
                else{
                    LEDSegment.BRStrip.setColor(stateColor);
                    LEDSegment.BLStrip.setColor(statusColor);
                }
            }
            //Facing right
            else if(SwerveDriveSubsystem.getInstance().getGyroAngle() > Constants.TAU/8
                &&  SwerveDriveSubsystem.getInstance().getGyroAngle() < Constants.TAU*3/8){
                LEDSegment.FLStrip.setColor(stateColor);
                LEDSegment.BLStrip.setColor(stateColor);

                if(DriverStation.getAlliance() == Alliance.Blue){
                    LEDSegment.BRStrip.setColor(stateColor);
                    LEDSegment.FRStrip.setColor(statusColor);
                }
                else{
                    LEDSegment.FRStrip.setColor(stateColor);
                    LEDSegment.BRStrip.setColor(statusColor);
                }
            }
            //Facing back
            else if(SwerveDriveSubsystem.getInstance().getGyroAngle() >= Constants.TAU*3/8
                &&  SwerveDriveSubsystem.getInstance().getGyroAngle() <= -Constants.TAU*3/8){
                LEDSegment.BLStrip.setColor(stateColor);
                LEDSegment.BRStrip.setColor(stateColor);
                
                if(DriverStation.getAlliance() == Alliance.Blue){
                    LEDSegment.FRStrip.setColor(stateColor);
                    LEDSegment.FLStrip.setColor(statusColor);
                }
                else{
                    LEDSegment.FLStrip.setColor(stateColor);
                    LEDSegment.FRStrip.setColor(statusColor);
                }
            }
            //Facing right
            else{
                LEDSegment.FRStrip.setColor(stateColor);
                LEDSegment.BRStrip.setColor(stateColor);

                if(DriverStation.getAlliance() == Alliance.Blue){
                    LEDSegment.FLStrip.setColor(stateColor);
                    LEDSegment.BLStrip.setColor(statusColor);
                }
                else{
                    LEDSegment.BLStrip.setColor(stateColor);
                    LEDSegment.FLStrip.setColor(statusColor);
                }
            }
        }
        else{
            
        }
    }

    public static void changeState(Color color){
        stateColor = color;
    }
    public static void changeStatus(Color color){
        statusColor = color;
    }
    public static void setFull(boolean f){
        full = f;
    }

    public static class Color{
        int r;
        int g;
        int b;

        public Color(int r, int g, int b){
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    public enum LEDSegment{
        ArmPositionIndicator(0, 2, 0),
        PivotEncoderIndicator(2, 2, 0),
        SwerveEncoderIndicator(4, 2, 0),
        VoltageIndicator(6, 2, 0),
        FLStrip(0, 0, 0),
        FRStrip(0, 0, 0),
        BRStrip(0, 0, 0),
        BLStrip(0, 0, 0),
        MainStrip(0, 0, 0);

        public final int startIndex;
        public final int segmentSize;
        public final int animationSlot;

        private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
        }

        public void setColor(Color color) {
            clearAnimation();
            candle.setLEDs(color.r, color.g, color.b, 0, startIndex, segmentSize);
        }

        private void setAnimation(Animation animation) {
            candle.animate(animation, animationSlot);
        }

        public void fullClear() {
            clearAnimation();
            disableLEDs();
        }

        public void clearAnimation() {
            candle.clearAnimation(animationSlot);
        }

        public void disableLEDs() {
            setColor(black);
        }

        public void setFlowAnimation(Color color, double speed) {
            setAnimation(new ColorFlowAnimation(
                    color.r, color.g, color.b, 0, speed, segmentSize, Direction.Forward, startIndex));
        }

        public void setFadeAnimation(Color color, double speed) {
            setAnimation(
                    new SingleFadeAnimation(color.r, color.g, color.b, 0, speed, segmentSize, startIndex));
        }

        public void setBoucneAnimation(Color color, double speed) {
            setAnimation(new LarsonAnimation(
                    color.r, color.g, color.b, 0, speed, segmentSize, BounceMode.Front, 3, startIndex));
        }

        public void setStrobeAnimation(Color color, double speed) {
            setAnimation(new StrobeAnimation(color.r, color.g, color.b, 0, speed, segmentSize, startIndex));
        }

        public void setRainbowAnimation(double speed) {
            setAnimation(new RainbowAnimation(1, speed, segmentSize, false, startIndex));
        }
    }
}
