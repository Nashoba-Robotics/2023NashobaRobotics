// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.SwerveState;

// hello
public class SwerveDriveSubsystem extends SubsystemBase {

    private SwerveModule mod1;
    private SwerveModule mod2;
    private SwerveModule mod3;
    private SwerveModule mod4;

    private Pigeon2 gyro;

    private SwerveDriveSubsystem() {     
        mod1 = new SwerveModule();   
        mod2 = new SwerveModule();
        mod3 = new SwerveModule();
        mod4 = new SwerveModule();

        gyro = new Pigeon2(0);
    }
    
    private static SwerveDriveSubsystem instance;

    public static SwerveDriveSubsystem getInstance() {
        if(instance == null) {
            instance = new SwerveDriveSubsystem();
        }
        return instance;
    }

    private double getGyroAngle() {
        return gyro.getYaw();
    }

    public void set(double x, double y, double omega, boolean fieldCentric) {

        double r = Math.sqrt(x*x + y*y);
        double tempAngle = Math.atan2(y, x);

        if(fieldCentric) {
            x = r * Math.cos(tempAngle + getGyroAngle());
            y = r * Math.sin(tempAngle + getGyroAngle());
        }

        x = r * Math.cos(tempAngle + 0);
        y = r * Math.sin(tempAngle + 0);

        //Repeated equations
        double h = omega * Constants.Swerve.width/2;
        double b = omega * Constants.Swerve.length/2;

        //The addition of the movement and rotational vector
        double x1 = x - b;
        double y1 = y + h;

        double x2 = x - b;
        double y2 = y - h;

        double x3 = x + b;
        double y3 = y - h;

        double x4 = x + b;
        double y4 = y + h;

        //Convert to polar
        SwerveState s1 = new SwerveState(
            Math.sqrt(x1*x1 + y1*y1),
            Math.atan2(y1, x1),
            0
            );

        SwerveState s2 = new SwerveState(
            Math.sqrt(x2*x2 + y2*y2),
            Math.atan2(y2, x2),
            0
        );

        SwerveState s3 = new SwerveState(
            Math.sqrt(x3*x3 + y3*y3),
            Math.atan2(y3, x3),
            0
        );

        SwerveState s4 = new SwerveState(
            Math.sqrt(x4*x4 + y4*y3),
            Math.atan2(y4, x4),
            0
        );

        mod1.set(12 * s1.move, s1.turn);
        mod2.set(12 * s2.move, s2.turn);
        mod3.set(12 * s3.move, s3.turn);
        mod4.set(12 * s4.move, s4.turn);

    }

    public void setTest(double move, double turn) {
        mod1.set(12 * move, turn);
        mod2.set(12 * move, turn);
        mod3.set(12 * move, turn);
        mod4.set(12 * move, turn);
    }
}
