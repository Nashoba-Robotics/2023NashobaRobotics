package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SwerveDriveCommand extends CommandBase {
    boolean squareUp;
    PIDController controller;

    public SwerveDriveCommand() {
        addRequirements(
            SwerveDriveSubsystem.getInstance(),
            JoystickSubsystem.getInstance()
        );

        controller = new PIDController(1.2, 0, 0.01);
        controller.setSetpoint(0);
        controller.setTolerance(Constants.TAU/100);
        controller.enableContinuousInput(-Constants.TAU/2, Constants.TAU/2);

        squareUp = false;
    }

    @Override
    public void execute() {
        if(JoystickSubsystem.getInstance().getRightButtonValue(3)) SwerveDriveSubsystem.getInstance().setFieldCentric(true);
        if(JoystickSubsystem.getInstance().getLeftButtonValue(4)) SwerveDriveSubsystem.getInstance().setFieldCentric(false);

        if(JoystickSubsystem.getInstance().getRightButtonValue(1)){
            double angle = SwerveDriveSubsystem.getInstance().getGyroAngle();
            controller.setSetpoint(
                angle < Constants.TAU/4 &&
                angle > -Constants.TAU/4 ? 
                0:
                Constants.TAU/2
            );
            squareUp = true;
        }

        if(squareUp) {
            SwerveDriveSubsystem.getInstance().set(
                JoystickSubsystem.getInstance().getLeftJoystickValues().shape(
                    Constants.Joystick.MOVE_DEAD_ZONE,
                    Constants.Joystick.MOVE_SENSITIVITY
                ).swap().applyAngleDeadzone(10 * Constants.TAU/360),
                -controller.calculate(SwerveDriveSubsystem.getInstance().getGyroAngle())
                );
        } else {
            SwerveDriveSubsystem.getInstance().set(
                JoystickSubsystem.getInstance().getLeftJoystickValues().shape(
                    Constants.Joystick.MOVE_DEAD_ZONE,
                    Constants.Joystick.MOVE_SENSITIVITY
                ).swap().applyAngleDeadzone(10 * Constants.TAU/360),
                JoystickSubsystem.getInstance().getRightJoystickValues().shape(
                    Constants.Joystick.TURN_DEAD_ZONE,
                    Constants.Joystick.TURN_SENSITIVITY
                ).x
                );
        }

        if(controller.atSetpoint() || JoystickSubsystem.getInstance().getRightJoystickValues().x > 0.5){
            squareUp = false;
        } 
    }
    @Override
    public void end(boolean interrupted) {
        // SwerveDriveSubsystem.getInstance().set(0, 0, 0);
    }
 
    @Override
    public boolean isFinished() {
        return false;
    }

}