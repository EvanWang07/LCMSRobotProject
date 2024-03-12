package frc.robot.commands;

import frc.robot.subsystems.Jukebox;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class TeleopJukebox extends Command {
    private Jukebox j_Jukebox;
    private DoubleSupplier shooterSpeedSup;
    private BooleanSupplier useIntake;
    
    public TeleopJukebox(Jukebox j_Jukebox, DoubleSupplier shooterSpeedSup, BooleanSupplier useIntake) {
        this.j_Jukebox = j_Jukebox;
        addRequirements(j_Jukebox);

        this.shooterSpeedSup = shooterSpeedSup;
        this.useIntake = useIntake;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double j_shooterSpeed = shooterSpeedSup.getAsDouble();
        boolean j_useIntake = useIntake.getAsBoolean();
        if (j_useIntake) {
            if (j_shooterSpeed > 0) {
                j_Jukebox.setIntakeMotorSpeeds(1.0, true);
            } else {
                if (!(j_Jukebox.checkSensorBreakage())) {
                    j_Jukebox.setIntakeMotorSpeeds(1.0, false);
                }
            }
        } else {
            j_Jukebox.brakeIntakeMotors();
        }
        j_Jukebox.setShooterMotorSpeeds(j_shooterSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        j_Jukebox.brakeIntakeMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
