package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Time extends SubsystemBase {
    public Timer weaponTimer;

    public Time() {
        weaponTimer = new Timer();
        weaponTimer.start();
    }

    public double getTime() {
        return weaponTimer.get();
    }

    public void resetTime() {
        weaponTimer.reset();
    }

    @Override
    public void periodic() {
        SmartDashboard.getNumber("Weapon Timer", getTime());
    }
}
