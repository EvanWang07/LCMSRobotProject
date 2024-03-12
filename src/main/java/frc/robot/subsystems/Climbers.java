package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climbers extends SubsystemBase {
    public final VoltageOut Climber_request = new VoltageOut(0);
    public TalonFX leftClimberMotor;
    public TalonFX rightClimberMotor;

    public Climbers() {
        leftClimberMotor = new TalonFX(Constants.Climbers.leftClimberMotorID, Constants.Drive.CANBusName);
        rightClimberMotor = new TalonFX(Constants.Climbers.rightClimberMotorID, Constants.Drive.CANBusName);

        leftClimberMotor.setInverted(Constants.Climbers.leftClimberMotorInverted);
        rightClimberMotor.setInverted(Constants.Climbers.rightClimberMotorInverted);
    }

    public void setClimberMotorSpeeds(double speed) {
        leftClimberMotor.setControl(Climber_request.withOutput(speed * Constants.Climbers.climbersMaxVoltage * Constants.Drive.basePercentClimberOutput));
        rightClimberMotor.setControl(Climber_request.withOutput(speed * Constants.Climbers.climbersMaxVoltage * Constants.Drive.basePercentClimberOutput));
    }

    public void brakeClimberMotors() {
        // leftClimberMotor.setControl(Climber_request.withOutput(0));
        // rightClimberMotor.setControl(Climber_request.withOutput(0));
        leftClimberMotor.setNeutralMode(NeutralModeValue.Brake);
        rightClimberMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    // @Override
    // public void periodic() {

    // }
}
