package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NetworkTables extends SubsystemBase {

    private static NetworkTable clawCam = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    public NetworkTables() {}

    public double getAngle() {
        return clawCam.getEntry("angle").getDouble(0);
    }
}
