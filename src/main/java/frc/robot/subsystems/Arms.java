package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;

public class Arms extends SubsystemBase {
    public final VoltageOut Arm_request = new VoltageOut(0);
    MotionMagicVoltage Magic_arm_request = new MotionMagicVoltage(0);
    public TalonFX leftArm;
    public TalonFX rightArm;
    public double PIDTargetPosition;

    public Arms() {
        leftArm = new TalonFX(Constants.Arms.leftArmMotorID, Constants.Drive.CANBusName);
        rightArm = new TalonFX(Constants.Arms.rightArmMotorID, Constants.Drive.CANBusName);

        leftArm.getConfigurator().apply(Robot.ctreConfigs.armTalonFXConfigs);
        rightArm.getConfigurator().apply(Robot.ctreConfigs.armTalonFXConfigs);

        new Thread(() -> {
        try {
            Thread.sleep(1000);
            leftArm.setInverted(Constants.Arms.leftArmMotorInverted);
            rightArm.setInverted(Constants.Arms.rightArmMotorInverted);
            leftArm.setPosition(Units.degreesToRotations(Constants.Drive.calculatedArmMotorStartPosition));
            rightArm.setPosition(Units.degreesToRotations(Constants.Drive.calculatedArmMotorStartPosition));
            if (Constants.Display.showArmTheta) {
                SmartDashboard.putNumber("Left Arm Position", (getLeftArmPosition() / Constants.Arms.armMotorGearRatio));
                SmartDashboard.putNumber("Right Arm Position", (getRightArmPosition() / Constants.Arms.armMotorGearRatio));
                SmartDashboard.putNumber("Average Arm Position", (getAverageArmPosition() / Constants.Arms.armMotorGearRatio));
                SmartDashboard.putNumber("Arm Position Discrepancy", Math.abs((getLeftArmPosition() - getRightArmPosition()) / Constants.Arms.armMotorGearRatio));
            }
            if (Constants.Display.showArmDebugInfo) {
                SmartDashboard.putBoolean("Left Arm Invert", leftArm.getInverted());
                SmartDashboard.putBoolean("Right Arm Invert", rightArm.getInverted());
            }
        } catch (Exception e) {
            
        }
        }).start();
    }

    public double getScaledPercentArmOutput(double oldOutput) {
        double angleBoundsAdapter = 360 / Constants.Arms.armsMaximumRotation;
        double armOutputMultiplier = 0.075 + 0.925 * Math.abs(Math.sin(Math.toRadians(0.5 * (getAverageArmPosition() / Constants.Arms.armMotorGearRatio) * angleBoundsAdapter)));
        double newArmOutput = armOutputMultiplier * oldOutput;
        return newArmOutput;
    }

    public void setArmMotorSpeeds(double speed) {
        if ((Constants.Arms.leftArmMotorInverted == leftArm.getInverted()) && (Constants.Arms.rightArmMotorInverted == rightArm.getInverted())) {
            leftArm.setControl(Arm_request.withOutput(speed * Constants.Arms.armsMaxVoltage * getScaledPercentArmOutput(Constants.Drive.maxBasePercentArmOutput)));
            rightArm.setControl(Arm_request.withOutput(speed * Constants.Arms.armsMaxVoltage * getScaledPercentArmOutput(Constants.Drive.maxBasePercentArmOutput)));
        } else {
            System.out.println("[Arms] ERROR: Arm motor inverts are incorrect!");
        }
    }

    public void setLeftArmMotorSpeed(double speed) { // For manual arm calibration
        if ((Constants.Arms.leftArmMotorInverted == leftArm.getInverted()) && (Constants.Arms.rightArmMotorInverted == rightArm.getInverted())) {
            leftArm.setControl(Arm_request.withOutput(speed * Constants.Arms.armsMaxVoltage * getScaledPercentArmOutput(Constants.Drive.maxBasePercentArmOutput)));
        } else {
            System.out.println("[Left Arm] ERROR: Arm motor inverts are incorrect!");
        }
    }

    public void setRightArmMotorSpeed(double speed) { // For manual arm calibration
        if ((Constants.Arms.leftArmMotorInverted == leftArm.getInverted()) && (Constants.Arms.rightArmMotorInverted == rightArm.getInverted())) {
            rightArm.setControl(Arm_request.withOutput(speed * Constants.Arms.armsMaxVoltage * getScaledPercentArmOutput(Constants.Drive.maxBasePercentArmOutput)));
        } else {
            System.out.println("[Right Arm] ERROR: Arm motor inverts are incorrect!");
        }
    }

    public void brakeArmMotors(boolean useNeutralMode) {
        if (useNeutralMode) {
            leftArm.setNeutralMode(NeutralModeValue.Brake);
            rightArm.setNeutralMode(NeutralModeValue.Brake);
        } else {
            leftArm.setControl(Arm_request.withOutput(0));
            rightArm.setControl(Arm_request.withOutput(0));
        }
        // leftArm.stopMotor();
        // rightArm.stopMotor();
    }

    public void brakeLeftArmMotor(boolean useNeutralMode) { // For manual arm calibration
        if (useNeutralMode) {
            leftArm.setNeutralMode(NeutralModeValue.Brake);
        } else {
            leftArm.setControl(Arm_request.withOutput(0));
        }
        // leftArm.stopMotor();
    }

    public void brakeRightArmMotor(boolean useNeutralMode) { // For manual arm calibration
        if (useNeutralMode) {
            rightArm.setNeutralMode(NeutralModeValue.Brake);
        } else {
            rightArm.setControl(Arm_request.withOutput(0));
        }
        // rightArm.stopMotor();
    }

    public double getLeftArmPosition() {
        var leftPositionSignal = leftArm.getPosition();
        double leftAnglePosition = leftPositionSignal.getValueAsDouble() + Constants.Arms.calculatedLeftArmThetaOffset;
        return Units.rotationsToDegrees(leftAnglePosition);
    }

    public double getRightArmPosition() {
        var rightPositionSignal = rightArm.getPosition();
        double rightAnglePosition = rightPositionSignal.getValueAsDouble() + Constants.Arms.calculatedRightArmThetaOffset;
        return Units.rotationsToDegrees(rightAnglePosition);
    }

    public double getAverageArmPosition() {
        double averageArmPosition = (getLeftArmPosition() + getRightArmPosition()) / 2;
        return averageArmPosition;
    }

    public void newAutoSetArmPosition(double setPoint, double minimumSpeed) {
        if ((Constants.Arms.leftArmMotorInverted == leftArm.getInverted()) && (Constants.Arms.rightArmMotorInverted == rightArm.getInverted())) {
            if (Math.abs(getLeftArmPosition() - getRightArmPosition()) <= Constants.Arms.armsMaxErrorTolerance) { // Checks if the motors are synchronized
                double armPIDError = Math.abs(setPoint - getAverageArmPosition());
                double armPIDRotationVelocity = armPIDError * Constants.Arms.armHeldKP;
                double rotationalAdjustment = 0;
                while (armPIDError > Constants.Arms.calculatedMaxHeldPIDArmThetaOffset) {
                    if (setPoint < getAverageArmPosition()) {
                        rotationalAdjustment = -armPIDRotationVelocity - minimumSpeed;
                    } else {
                        rotationalAdjustment = armPIDRotationVelocity + minimumSpeed;
                    }

                    setArmMotorSpeeds(rotationalAdjustment);
                }
                if (((Math.abs(setPoint - Constants.Arms.armLowerBoundTheta)) <= (5 * Constants.Arms.armMotorGearRatio)) || ((Math.abs(Constants.Arms.armUpperBoundTheta - setPoint)) <= (5 * Constants.Arms.armMotorGearRatio))) {
                    brakeArmMotors(false);
                } else {
                    brakeArmMotors(true);
                }
            } else {
                System.out.println("[PID] WARNING: Arms need calibration!");
            }
        } else {
            System.out.println("[Arms] ERROR: Arm motor inverts are incorrect!");
        }
    }

    public void motionMagicAutoSetArmPosition(double setPoint) {
        PIDTargetPosition = setPoint / Constants.Arms.armMotorGearRatio;
        double setPointInRotations = setPoint / 360;
        double setToleranceInRotations = Constants.Arms.calculatedMaxPIDArmThetaOffset / 360;

        if ((Constants.Arms.leftArmMotorInverted == leftArm.getInverted()) && (Constants.Arms.rightArmMotorInverted == rightArm.getInverted())) {
            System.out.println("[Magic PID] Rotating the arm motors to " + (setPoint / Constants.Arms.armMotorGearRatio) + " degrees!");
            if (Constants.Display.showArmDebugInfo) {
                SmartDashboard.putNumber("PID Target Position", PIDTargetPosition);
            }
            while ((Magic_arm_request.Position < (setPointInRotations - setToleranceInRotations)) || (Magic_arm_request.Position > (setPointInRotations + setToleranceInRotations))) {
                leftArm.setControl(Magic_arm_request.withPosition(setPointInRotations));
                rightArm.setControl(Magic_arm_request.withPosition(setPointInRotations));
                System.out.println("[Magic PID] Running. Current position: " + getAverageArmPosition() / Constants.Arms.armMotorGearRatio);
            }
            if (((Math.abs(setPoint - Constants.Arms.armLowerBoundTheta)) <= (5 * Constants.Arms.armMotorGearRatio)) || ((Math.abs(Constants.Arms.armUpperBoundTheta - setPoint)) <= (5 * Constants.Arms.armMotorGearRatio))) {
                brakeArmMotors(false);
            } else {
                brakeArmMotors(true);
            }
            System.out.println("[Magic PID] Finished rotating the arm motors to " + (setPoint / Constants.Arms.armMotorGearRatio) + " degrees!");
            System.out.println("[Magic PID] An error of " + (Math.abs(getAverageArmPosition() - setPoint) / Constants.Arms.armMotorGearRatio) + " degree(s) was detected!");
        } else {
            System.out.println("[Arms] ERROR: Arm motor inverts are incorrect!");
        }
    }

    public void autoSetArmPosition(double setPoint) {
        PIDTargetPosition = setPoint / Constants.Arms.armMotorGearRatio;
        PIDController a_PIDController = new PIDController(Constants.Arms.armKP, Constants.Arms.armKI, Constants.Arms.armKD);

        a_PIDController.setSetpoint(setPoint);
        a_PIDController.setTolerance(Constants.Arms.calculatedMaxPIDArmThetaOffset);
        a_PIDController.setIZone(Constants.Arms.calculatedMaxPIDArmIntegrationZone);
        if ((Constants.Arms.leftArmMotorInverted == leftArm.getInverted()) && (Constants.Arms.rightArmMotorInverted == rightArm.getInverted())) {
            if (Math.abs(getLeftArmPosition() - getRightArmPosition()) <= Constants.Arms.armsMaxErrorTolerance) { // Checks if the motors are synchronized
                System.out.println("[PID] Rotating the arm motors to " + (setPoint / Constants.Arms.armMotorGearRatio) + " degrees!");
                if (Constants.Display.showArmDebugInfo) {
                    SmartDashboard.putNumber("PID Target Position", PIDTargetPosition);
                }
                a_PIDController.reset();
                while (!(a_PIDController.atSetpoint())) {
                    double newArmSpeed = a_PIDController.calculate(getAverageArmPosition());
                    setArmMotorSpeeds(newArmSpeed * Constants.Arms.percentAutomaticArmOutput);
                    System.out.println("[PID] Running. Current position: " + getAverageArmPosition() / Constants.Arms.armMotorGearRatio);
                }
                if (((Math.abs(setPoint - Constants.Arms.armLowerBoundTheta)) <= (5 * Constants.Arms.armMotorGearRatio)) || ((Math.abs(Constants.Arms.armUpperBoundTheta - setPoint)) <= (5 * Constants.Arms.armMotorGearRatio))) {
                    brakeArmMotors(false);
                } else {
                    brakeArmMotors(true);
                }
                a_PIDController.close();
                System.out.println("[PID] Finished rotating the arm motors to " + (setPoint / Constants.Arms.armMotorGearRatio) + " degrees!");
                System.out.println("[PID] An error of " + (Math.abs(getAverageArmPosition() - setPoint) / Constants.Arms.armMotorGearRatio) + " degree(s) was detected!");
            } else {
                System.out.println("[PID] WARNING: Arms need calibration!");
            }
        } else {
            System.out.println("[Arms] ERROR: Arm motor inverts are incorrect!");
        }
    }

    public void autoSetLeftArmPosition(double setPoint) {
        PIDTargetPosition = setPoint / Constants.Arms.armMotorGearRatio;
        PIDController a_PIDController = new PIDController(Constants.Arms.armKP, Constants.Arms.armKI, Constants.Arms.armKD);
        
        a_PIDController.setSetpoint(setPoint);
        a_PIDController.setTolerance(Constants.Arms.calculatedMaxPIDArmThetaOffset);
        a_PIDController.setIZone(Constants.Arms.calculatedMaxPIDArmIntegrationZone);
        if ((Constants.Arms.leftArmMotorInverted == leftArm.getInverted()) && (Constants.Arms.rightArmMotorInverted == rightArm.getInverted())) {
            if (Math.abs(getLeftArmPosition() - getRightArmPosition()) <= Constants.Arms.armsMaxErrorTolerance) { // Checks if the motors are synchronized
                System.out.println("[PID] Rotating the left arm motor to " + (setPoint / Constants.Arms.armMotorGearRatio) + " degrees!");
                a_PIDController.reset();
                while (!(a_PIDController.atSetpoint())) {
                    double newLeftArmSpeed = a_PIDController.calculate(getLeftArmPosition());
                    setArmMotorSpeeds(newLeftArmSpeed * Constants.Arms.percentAutomaticArmOutput);
                    System.out.println("[PID] Running. Current left arm position: " + getLeftArmPosition() / Constants.Arms.armMotorGearRatio);
                }
                if (((Math.abs(setPoint - Constants.Arms.armLowerBoundTheta)) <= (5 * Constants.Arms.armMotorGearRatio)) || ((Math.abs(Constants.Arms.armUpperBoundTheta - setPoint)) <= (5 * Constants.Arms.armMotorGearRatio))) {
                    brakeLeftArmMotor(false);
                } else {
                    brakeLeftArmMotor(true);
                }
                a_PIDController.close();
                System.out.println("[PID] Finished rotating the left arm motor to " + (setPoint / Constants.Arms.armMotorGearRatio) + " degrees!");
                System.out.println("[PID] An error of " + (Math.abs(getLeftArmPosition() - setPoint) / Constants.Arms.armMotorGearRatio) + " degree(s) was detected!");
            } else {
                System.out.println("[PID Left] WARNING: Arms need calibration!");
            }
        } else {
            System.out.println("[Left Arm] ERROR: Arm motor inverts are incorrect!");
        }
    }

    public void autoSetRightArmPosition(double setPoint) {
        PIDTargetPosition = setPoint / Constants.Arms.armMotorGearRatio;
        PIDController a_PIDController = new PIDController(Constants.Arms.armKP, Constants.Arms.armKI, Constants.Arms.armKD);
        
        a_PIDController.setSetpoint(setPoint);
        a_PIDController.setTolerance(Constants.Arms.calculatedMaxPIDArmThetaOffset);
        a_PIDController.setIZone(Constants.Arms.calculatedMaxPIDArmIntegrationZone);
        if ((Constants.Arms.leftArmMotorInverted == leftArm.getInverted()) && (Constants.Arms.rightArmMotorInverted == rightArm.getInverted())) {
            if (Math.abs(getLeftArmPosition() - getRightArmPosition()) <= Constants.Arms.armsMaxErrorTolerance) { // Checks if the motors are synchronized
                System.out.println("[PID] Rotating the right arm motor to " + (setPoint / Constants.Arms.armMotorGearRatio) + " degrees!");
                a_PIDController.reset();
                while (!(a_PIDController.atSetpoint())) {
                    double newRightArmSpeed = a_PIDController.calculate(getRightArmPosition());
                    setArmMotorSpeeds(newRightArmSpeed * Constants.Arms.percentAutomaticArmOutput);
                    System.out.println("[PID] Running. Current right arm position: " + getRightArmPosition() / Constants.Arms.armMotorGearRatio);
                }
                if (((Math.abs(setPoint - Constants.Arms.armLowerBoundTheta)) <= (5 * Constants.Arms.armMotorGearRatio)) || ((Math.abs(Constants.Arms.armUpperBoundTheta - setPoint)) <= (5 * Constants.Arms.armMotorGearRatio))) {
                    brakeRightArmMotor(false);
                } else {
                    brakeRightArmMotor(true);
                }
                a_PIDController.close();
                System.out.println("[PID] finished rotating the right arm motor to " + (setPoint / Constants.Arms.armMotorGearRatio) + " degrees!");
                System.out.println("[PID] An error of " + (Math.abs(getRightArmPosition() - setPoint) / Constants.Arms.armMotorGearRatio) + " degree(s) was detected!");
            } else {
                System.out.println("[PID Right] WARNING: Arms need calibration!");
            }
        } else {
            System.out.println("[Right Arm] ERROR: Arm motor inverts are incorrect!");
        }
    }

    public void displayArmInfoSnapshot() {
        if (Constants.Display.showArmTheta) {
            SmartDashboard.putNumber("Left Arm Position", (getLeftArmPosition() / Constants.Arms.armMotorGearRatio));
            SmartDashboard.putNumber("Right Arm Position", (getRightArmPosition() / Constants.Arms.armMotorGearRatio));
            SmartDashboard.putNumber("Average Arm Position", (getAverageArmPosition() / Constants.Arms.armMotorGearRatio));
            SmartDashboard.putNumber("Arm Position Discrepancy", Math.abs((getLeftArmPosition() - getRightArmPosition()) / Constants.Arms.armMotorGearRatio));
        }
        if (Constants.Display.showArmDebugInfo) {
            SmartDashboard.putBoolean("Left Arm Invert", leftArm.getInverted());
            SmartDashboard.putBoolean("Right Arm Invert", rightArm.getInverted());
        }
    }

    // @Override
    // public void periodic() {

    // }
}
