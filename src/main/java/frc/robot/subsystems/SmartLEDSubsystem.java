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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SmartLEDSubsystem extends SubsystemBase{
    private static CANdle candle;
    public static LightState state;

    private LightState lastState = LightState.DEFAULT;
    private boolean cleared = false;

    public enum LightState{
        DEFAULT,
        WANT_CONE,
        HAVE_CONE,
        WANT_CUBE,
        HAVE_CUBE,
        SINGLE_STATION,
        DISABLED
    }

    public static final Color white = new Color(255, 255, 255);
    public static final Color black = new Color(0, 0, 0);
    public static final Color yellow = new Color(255, 80, 0);
    public static final Color purple = new Color(148, 0, 211);
    public static final Color green = new Color(0, 255, 0);
    public static final Color red = new Color(255, 0, 0);
    public static final Color orange = new Color(0xFF, 0x10, 0x0);
    public static final Color blue = new Color(0, 0, 255);

    public SmartLEDSubsystem(){
        candle = new CANdle(Constants.Misc.CANDLE_PORT, "drivet");
        state = LightState.DEFAULT;
    }
    private static boolean full = false; // Represents whether or not the full LED stips are lit up
    private static Color statusColor = green;   // Whether the robot has a bad thing going on
    private static Color stateColor = white;    // Light to represent what the robot wants
    @Override
    public void periodic() {
        if(state != lastState){
            fullClear();
        }
        switch(state){
            case DISABLED:
                stateColor = orange;
                break;
            case DEFAULT:
                stateColor = white;
                break;
            case WANT_CONE:
                stateColor = yellow;
                break;
            case WANT_CUBE:
                stateColor = purple;
                break;
            case HAVE_CONE:
                stateColor = green;
                break;
            case HAVE_CUBE:
                stateColor = green;
                break;
            case SINGLE_STATION:
                stateColor = blue;
                break;
            default:
                stateColor = white;
                break;
        }

        if(DriverStation.isEnabled()){
            //Front facing away from DS
            if(!full){
                // LEDSegment.MainStrip.clearAnimation();
                if(SwerveDriveSubsystem.getInstance().getGyroAngle() >= -Constants.TAU/8
                && SwerveDriveSubsystem.getInstance().getGyroAngle() <= Constants.TAU/8){
                    SmartDashboard.putNumber("Light", 0);

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
                else if(SwerveDriveSubsystem.getInstance().getGyroAngle() < -Constants.TAU/8
                    &&  SwerveDriveSubsystem.getInstance().getGyroAngle() > -Constants.TAU*3/8){
                    SmartDashboard.putNumber("Light", 1);

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
                    ||  SwerveDriveSubsystem.getInstance().getGyroAngle() <= -Constants.TAU*3/8){
                    SmartDashboard.putNumber("Light", 2);
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
                //Facing Left
                else{
                    SmartDashboard.putNumber("Light", 3);

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
                LEDSegment.MainStrip.setColor(stateColor);
            }
        }
        else if(DriverStation.isDisabled()){
            LEDSegment.MainStrip.setBoucneAnimation(orange, 0.1);
        }
        else if(DriverStation.isAutonomous()){
            LEDSegment.MainStrip.setRainbowAnimation(0.1);
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

    public static void fullClear(){
        LEDSegment.MainStrip.fullClear();

        LEDSegment.BLStrip.fullClear();
        LEDSegment.BRStrip.fullClear();
        LEDSegment.FLStrip.fullClear();
        LEDSegment.FRStrip.fullClear();
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
        FLStrip(8, 27, 0),//8-35
        FRStrip(89, 27, 0),
        BRStrip(61, 28, 0),
        BLStrip(35, 26, 0),
        MainStrip(8, 108, 0);

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
                    color.r, color.g, color.b, 0, speed, segmentSize, BounceMode.Front, 7, startIndex));
        }

        public void setStrobeAnimation(Color color, double speed) {
            setAnimation(new StrobeAnimation(color.r, color.g, color.b, 0, speed, segmentSize, startIndex));
        }

        public void setRainbowAnimation(double speed) {
            setAnimation(new RainbowAnimation(1, speed, segmentSize, false, startIndex));
        }
    }
}
