package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.math.Units;
import frc.robot.lib.util.SwerveState;
import frc.robot.subsystems.SwerveModule;

public class SingleModuleCommand extends CommandBase {
    
    SwerveModule module;

    @Override
    public void initialize() {
        module = new SwerveModule(
            0,
            Constants.Swerve.FRONT_RIGHT_MOVE_PORT,
            Constants.Swerve.FRONT_RIGHT_TURN_PORT,
            Constants.Swerve.FRONT_RIGHT_SENSOR_PORT,
            0,
            Constants.Swerve.MOD0_AFF
        );
        // SmartDashboard.putNumber("after module", module.getTurnPosition());
        // module.setTurnMotor(
        //     17000
        // );
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("turn NU", module.getTurnPosition());
        SmartDashboard.putNumber("abs angle", module.getAbsAngle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
