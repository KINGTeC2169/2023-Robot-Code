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

    public static double[] getFrontCenter() {
        return getFrontCenter("Cone")[0] == 0 ? 
        getFrontCenter("Cube") : getFrontCenter("Cone");
    }

    public static String closestObject() {
        if(getFrontCenter("Cube")[1] != 0 || getFrontCenter("Cone")[1] != 0) {
            return table.getEntry("Front-Cone-Center").getDoubleArray(arr)[1] < table.getEntry("Front-Cone-Center").getDoubleArray(arr)[1] ? 
        "Cone" : "Cube";
        }
        else
            return "None";
    }

    public static String isThereObject() {
        if(getFrontCenter("Cone")[0] != 0 && getFrontCenter("Cube")[0] != 0)
            return "Both";
        else if(getFrontCenter("Cube")[0] != 0) 
            return "Cube";
        else if(getFrontCenter("Cone")[0] != 0)
            return "Cone";
        else
            return "None";
            
    }
        
}
