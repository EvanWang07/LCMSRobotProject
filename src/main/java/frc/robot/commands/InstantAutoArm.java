package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Time;

import edu.wpi.first.wpilibj2.command.Command;

public class InstantAutoArm extends Command {
    private Arms a_Arms;
    private Time t_Time;
    private double setPoint;
    private double endTime;

    public InstantAutoArm(Time t_Time, Arms a_Arms, double setPoint, double endTime) {
        this.a_Arms = a_Arms;
        addRequirements(a_Arms);
        this.t_Time = t_Time;
        addRequirements(t_Time);

        this.setPoint = setPoint;
        this.endTime = endTime;
    }

    @Override
    public void initialize() {
        t_Time.resetTime();
    }

    @Override
    public void execute() {
        double a_setPoint = setPoint;
        double armPIDError = Math.abs(a_setPoint - a_Arms.getAverageArmPosition());
        double armPIDRotationVelocity = armPIDError * Constants.Arms.armHeldKP;
        double rotationalAdjustment = 0;

        if (armPIDError > Constants.Arms.calculatedMaxHeldPIDArmThetaOffset) {
            if (Math.abs(a_Arms.getLeftArmPosition() - a_Arms.getRightArmPosition()) <= Constants.Arms.armsMaxErrorTolerance) { // Checks if the motors are synchronized
                if (a_setPoint < a_Arms.getAverageArmPosition()) {
                    rotationalAdjustment = -armPIDRotationVelocity - Constants.Arms.armHeldPIDMinimumRotationalMovement;
                } else {
                    rotationalAdjustment = armPIDRotationVelocity + Constants.Arms.armHeldPIDMinimumRotationalMovement;
                }
                a_Arms.setArmMotorSpeeds(rotationalAdjustment);
            } else {
                System.out.println("WARNING: Arms need calibration! [HeldAutoArm]");
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (((Math.abs(setPoint - Constants.Arms.armLowerBoundTheta)) <= (5 * Constants.Arms.armMotorGearRatio)) || ((Math.abs(Constants.Arms.armUpperBoundTheta - setPoint)) <= (5 * Constants.Arms.armMotorGearRatio))) {
            a_Arms.brakeArmMotors(false);
        } else {
            a_Arms.brakeArmMotors(true);
        }
    }

    @Override
    public boolean isFinished() {
        if (t_Time.getTime() <= endTime) {
            return false;
        } else {
            return true;
        }
    }
}
