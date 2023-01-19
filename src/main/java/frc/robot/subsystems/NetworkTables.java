package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTables {

    private static double[] arr = {0,0};
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    public NetworkTables() {}

    public static double[] getPalmCenter(String coneOrCube) {
        return table.getEntry("Palm-" + coneOrCube + "-Center").getDoubleArray(arr);
    }

    public static double getPalmAngle(String coneOrCube) {
        return table.getEntry("Palm-" + coneOrCube + "-Angle").getDouble(0);
    }

    public static double[] getFrontCenter(String coneOrCube) {
        return table.getEntry("Front-" + coneOrCube + "-Center").getDoubleArray(arr);
    }


}
