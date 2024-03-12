package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arms;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopArm extends Command {
    private Arms a_Arms;
    private DoubleSupplier speedSup;
    private BooleanSupplier slowModeSup;

    public TeleopArm(Arms a_Arms, DoubleSupplier speedSup, BooleanSupplier slowModeSup) {
        this.a_Arms = a_Arms;
        addRequirements(a_Arms);

        this.speedSup = speedSup;
        this.slowModeSup = slowModeSup;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double a_speed = MathUtil.applyDeadband(speedSup.getAsDouble(), Constants.Drive.armStickDeadband);
        boolean a_slowMode = slowModeSup.getAsBoolean();
        boolean a_leftArmCanMove = false;
        boolean a_rightArmCanMove = false;

        if (a_slowMode) {
            a_speed *= Constants.Drive.percentMaxBasePercentArmOutput;
        }

        if (Constants.Arms.armCalibrationMode) { // For calibrating the arm motors
            /* Left Arm Motor */
            if (a_Arms.getLeftArmPosition() <= Constants.Arms.armUpperBoundTheta && a_Arms.getLeftArmPosition() >= Constants.Arms.armLowerBoundTheta) {
                a_Arms.setLeftArmMotorSpeed(a_speed);
            } else if (a_Arms.getLeftArmPosition() > Constants.Arms.armUpperBoundTheta) {
                if (a_speed < 0) {
                    a_Arms.setLeftArmMotorSpeed(a_speed);
                } else {
                    a_Arms.brakeLeftArmMotor(false);
                }
            } else {
                if (a_speed > 0) {
                    a_Arms.setLeftArmMotorSpeed(a_speed);
                } else {
                    a_Arms.brakeLeftArmMotor(false);
                }
            }

            /* Right Arm Motor */
            if (a_Arms.getRightArmPosition() <= Constants.Arms.armUpperBoundTheta && a_Arms.getRightArmPosition() >= Constants.Arms.armLowerBoundTheta) {
                a_Arms.setRightArmMotorSpeed(a_speed);
            } else if (a_Arms.getRightArmPosition() > Constants.Arms.armUpperBoundTheta) {
                if (a_speed < 0) {
                    a_Arms.setRightArmMotorSpeed(a_speed);
                } else {
                    a_Arms.brakeRightArmMotor(false);
                }
            } else {
                if (a_speed > 0) {
                    a_Arms.setRightArmMotorSpeed(a_speed);
                } else {
                    a_Arms.brakeRightArmMotor(false);
                }
            }
        } else { // Both Left & Right Motors
            if (Math.abs(a_Arms.getLeftArmPosition() - a_Arms.getRightArmPosition()) <= Constants.Arms.armsMaxErrorTolerance) { // Checks if the motors are synchronized

                if (a_Arms.getLeftArmPosition() <= Constants.Arms.armUpperBoundTheta && a_Arms.getLeftArmPosition() >= Constants.Arms.armLowerBoundTheta) {
                    a_leftArmCanMove = true;
                } else if (a_Arms.getLeftArmPosition() > Constants.Arms.armUpperBoundTheta) {
                    if (a_speed < 0) {
                        a_leftArmCanMove = true;
                    } else {
                        a_leftArmCanMove = false;
                    }
                } else {
                    if (a_speed > 0) {
                        a_leftArmCanMove = true;
                    } else {
                        a_leftArmCanMove = false;
                    }
                }

                if (a_Arms.getRightArmPosition() <= Constants.Arms.armUpperBoundTheta && a_Arms.getRightArmPosition() >= Constants.Arms.armLowerBoundTheta) {
                    a_rightArmCanMove = true;
                } else if (a_Arms.getRightArmPosition() > Constants.Arms.armUpperBoundTheta) {
                    if (a_speed < 0) {
                        a_rightArmCanMove = true;
                    } else {
                        a_rightArmCanMove = false;
                    }
                } else {
                    if (a_speed > 0) {
                        a_rightArmCanMove = true;
                    } else {
                        a_rightArmCanMove = false;
                    }
                }

                if (a_leftArmCanMove && a_rightArmCanMove) {
                    a_Arms.setArmMotorSpeeds(a_speed);
                } else {
                    a_Arms.brakeArmMotors(false);
                }

            } else {
                a_Arms.brakeArmMotors(false);
                System.out.println("WARNING: Arms need calibration! [TeleopArm]");
            }
        }        
    }

    @Override
    public void end(boolean interrupted) {
        a_Arms.brakeArmMotors(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
     
}
