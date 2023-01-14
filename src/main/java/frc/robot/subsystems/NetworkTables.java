package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTables {

    private static double[] arr = {0,0};
    private static NetworkTable clawCam = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    public NetworkTables() {}

    public static double getAngle() {
        return clawCam.getEntry("angle").getDouble(0);
    }

    public static double[] getCenter() {
        return clawCam.getEntry("center").getDoubleArray(arr);
    }
}
