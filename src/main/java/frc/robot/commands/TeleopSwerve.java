package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;
    private Vision v_Vision;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier slowModeSup;
    private BooleanSupplier autoAlign;

    public TeleopSwerve(Swerve s_Swerve, Vision v_Vision, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier slowModeSup, BooleanSupplier autoAlign) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.v_Vision = v_Vision;
        addRequirements(v_Vision);

        this.slowModeSup = slowModeSup;
        this.autoAlign = autoAlign;
        this.robotCentricSup = robotCentricSup;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
    }

    @Override
    public void execute() {
        double headingAlignVelocity = v_Vision.getTX() * Constants.Vision.visionRotationKP;
        double distanceAlignVelocity = v_Vision.getTY() * Constants.Vision.visionTranslationKP;
        boolean s_slowMode = slowModeSup.getAsBoolean();
        boolean m_autoAlign = autoAlign.getAsBoolean();
        
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Drive.driveStickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Drive.driveStickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Drive.driveStickDeadband);

        if (s_slowMode) {
            translationVal *= Constants.Drive.percentBasePercentDriveOutput;
            strafeVal *= Constants.Drive.percentBasePercentDriveOutput;
            rotationVal *= Constants.Drive.percentBasePercentDriveOutput;
        }

        if (m_autoAlign) {
            if (v_Vision.hasTarget()) {
                double headingError = v_Vision.getTX();
                double distanceError = v_Vision.getTY();
                // System.out.println(v_Vision.getTX() + " | " + v_Vision.getTY());
                double rotationalAdjustment = 0.0;
                double translationalAdjustment = 0.0;
                boolean autoAlignCompleted = false;

                if (Math.abs(headingError) > Constants.Vision.maxHorizontalAngleAlignError) {
                    if (headingError < 0) {
                        rotationalAdjustment = headingAlignVelocity + Constants.Vision.visionMinimumRotationalMovement;
                    } else {
                        rotationalAdjustment = headingAlignVelocity - Constants.Vision.visionMinimumRotationalMovement;
                    }
                    s_Swerve.drive(
                        new Translation2d(0, 0), 
                        rotationalAdjustment * Constants.Swerve.maxAngularVelocity, 
                        false, 
                        false
                    );
                } else if (Math.abs(distanceError) > Constants.Vision.distanceAlignSetpoint) {
                    if (distanceError < 0) {
                        translationalAdjustment = distanceAlignVelocity + Constants.Vision.visionMinimumTranslationalMovement;
                    } else {
                        translationalAdjustment = distanceAlignVelocity - Constants.Vision.visionMinimumTranslationalMovement;
                    }
                    s_Swerve.drive(
                        new Translation2d(0, translationalAdjustment), 
                        0, 
                        false, 
                        false
                    );
                } else {
                    autoAlignCompleted = true;
                    SmartDashboard.putBoolean("Automatic Target Completed", autoAlignCompleted);
                }
            }
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}