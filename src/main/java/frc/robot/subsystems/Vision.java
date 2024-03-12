package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends SubsystemBase {
    private final NetworkTable m_limelightTable;
    private double tx;
    private double ty;
    private double ta;

    public Vision() {
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getTX() {
        tx = m_limelightTable.getEntry("tx").getDouble(0);
        return tx;
    }

    public double getTY() {
        ty = m_limelightTable.getEntry("ty").getDouble(0);
        return ty;
    }

    public double getTA() {
        ta = m_limelightTable.getEntry("ta").getDouble(0);
        return ta;
    }

    public boolean hasTarget() {
        NetworkTableEntry tv = m_limelightTable.getEntry("tv");
        m_limelightTable.getEntry("tv").getDouble(0);
        return tv.getDouble(0) >= 1.0; // tv represents whether it has a vision target (returns 0 or 1)
    }

    public void displayVisionInfoSnapshot() {
        if (Constants.Display.showHorizontalVisionError) {
            SmartDashboard.putNumber("Horizontal Offset", getTX());
        }
        if (Constants.Display.showVerticalVisionError) {
            SmartDashboard.putNumber("Vertical Offset", getTY());
        }
        if (Constants.Display.showDistanceVisionError) {
            SmartDashboard.putNumber("Frame Area", getTA()); // Might be unnecessary
        }
    }

    @Override
    public void periodic() {
        
    }
}
