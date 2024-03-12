package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Climbers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopClimb extends Command {
    private Climbers c_Climbers;
    private DoubleSupplier speedSup;
    
    public TeleopClimb(Climbers c_Climbers, DoubleSupplier speedSup) {
        this.c_Climbers = c_Climbers;
        addRequirements(c_Climbers);

        this.speedSup = speedSup;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double c_speed = MathUtil.applyDeadband(speedSup.getAsDouble(), Constants.Drive.climberTriggerDeadband);
        c_Climbers.setClimberMotorSpeeds(c_speed);
    }

    @Override
    public void end(boolean interrupted) {
        c_Climbers.brakeClimberMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
