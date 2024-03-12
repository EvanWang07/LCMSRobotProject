package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(Constants.Drive.driveController);

    /* Drive Controls */
    private final int d_translationAxis = XboxController.Axis.kLeftY.value;
    private final int d_strafeAxis = XboxController.Axis.kLeftX.value;
    private final int d_rotationAxis = XboxController.Axis.kRightX.value;

    private final int d_shooterAxis = XboxController.Axis.kRightTrigger.value;

    /* Driver Buttons */
    private final JoystickButton d_robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton d_zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    
    private final JoystickButton d_lowerArm = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton d_ampAutoArm = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton d_speakerAutoArm = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton d_intakeNote = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Arms a_Arms = new Arms();
    private final Jukebox j_Jukebox = new Jukebox();
    private final Vision v_Vision = new Vision();
    private final Time t_Time = new Time();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        /* PathPlanner Registered Commands */
        NamedCommands.registerCommand("Intake Note", new AutoIntake(t_Time, j_Jukebox, 2.25).withTimeout(2.25));
        NamedCommands.registerCommand("Shoot Note", new AutoJukebox(t_Time, j_Jukebox).withTimeout(2.5));
        NamedCommands.registerCommand("Arm to Amp", new InstantAutoArm(t_Time, a_Arms, Constants.Arms.calculatedArmThetaAtAmp, 2.25).withTimeout(2.25)); // NOT USED
        NamedCommands.registerCommand("Arm to Bottom", new InstantAutoArm(t_Time, a_Arms, 650, 0.75).withTimeout(0.75)); // 3 for single auto
        
        NamedCommands.registerCommand("Arm to Speaker", new InstantAutoArm(t_Time, a_Arms, Constants.Arms.calculatedArmThetaAtSpeaker, 3).withTimeout(3)); // 3 for single auto
        NamedCommands.registerCommand("Fast Arm to Speaker", new InstantAutoArm(t_Time, a_Arms, Constants.Arms.calculatedArmThetaAtSpeaker, 1.75).withTimeout(1.75));

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                v_Vision, 
                () -> -driver.getRawAxis(d_translationAxis), 
                () -> -driver.getRawAxis(d_strafeAxis), 
                () -> -driver.getRawAxis(d_rotationAxis), 
                () -> d_robotCentric.getAsBoolean(),
                () -> false,
                () -> false
            )
        );

        j_Jukebox.setDefaultCommand(
            new TeleopJukebox(
                j_Jukebox,
                () -> driver.getRawAxis(d_shooterAxis),  
                () -> d_intakeNote.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        d_zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        if (Constants.Drive.useInstantPID) {
            if (Constants.Drive.useMotionMagicPID) {
                d_lowerArm.onTrue(new InstantCommand(() -> a_Arms.motionMagicAutoSetArmPosition(Constants.Arms.calculatedArmThetaAtDefault)));
                d_ampAutoArm.onTrue(new InstantCommand(() -> a_Arms.motionMagicAutoSetArmPosition(Constants.Arms.calculatedArmThetaAtAmp)));
                d_speakerAutoArm.onTrue(new InstantCommand(() -> a_Arms.motionMagicAutoSetArmPosition(Constants.Arms.calculatedArmThetaAtSpeaker)));
            } else {
                if (Constants.Drive.useIndividualMotorPID) {
                    d_lowerArm.onTrue(new InstantCommand(() -> a_Arms.autoSetLeftArmPosition(Constants.Arms.calculatedArmThetaAtDefault)));
                    d_lowerArm.onTrue(new InstantCommand(() -> a_Arms.autoSetRightArmPosition(Constants.Arms.calculatedArmThetaAtDefault)));
                    d_ampAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetLeftArmPosition(Constants.Arms.calculatedArmThetaAtAmp)));
                    d_ampAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetRightArmPosition(Constants.Arms.calculatedArmThetaAtAmp)));
                    d_speakerAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetLeftArmPosition(Constants.Arms.calculatedArmThetaAtSpeaker)));
                    d_speakerAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetRightArmPosition(Constants.Arms.calculatedArmThetaAtSpeaker)));
                } else {
                    d_lowerArm.onTrue(new InstantCommand(() -> a_Arms.autoSetArmPosition(Constants.Arms.calculatedArmThetaAtDefault)));
                    d_ampAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetArmPosition(Constants.Arms.calculatedArmThetaAtAmp)));
                    d_speakerAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetArmPosition(Constants.Arms.calculatedArmThetaAtSpeaker)));
                }
            }
        } else {
            d_lowerArm.whileTrue(new HeldAutoArm(a_Arms, () -> Constants.Arms.calculatedArmThetaAtDefault));
            d_ampAutoArm.whileTrue(new HeldAutoArm(a_Arms, () -> Constants.Arms.calculatedArmThetaAtAmp));
            d_speakerAutoArm.whileTrue(new HeldAutoArm(a_Arms, () -> Constants.Arms.calculatedArmThetaAtSpeaker));
        }
    }

    public Command getAutonomousCommand() {
        // The PathPlanner path will run
        return new PathPlannerAuto("");
    }
}