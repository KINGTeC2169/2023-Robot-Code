package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTables {

    private static double[] arr = {-2169,-2169};
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    public NetworkTables() {}

    public static double[] getPalmCenter(String coneOrCube) {
        //returns in a array with [x,y]
        return table.getEntry("Palm-" + coneOrCube + "-Center").getDoubleArray(arr);
    }

    public static double getPalmAngle(String coneOrCube) {
        return table.getEntry("Palm-" + coneOrCube + "-Angle").getDouble(-2169);
    }

    public static double[] getFrontCenter(String coneOrCube) {
        return table.getEntry("Front-" + coneOrCube + "-Center").getDoubleArray(arr);
    }

    public static double[] getFrontCenter() {
        return getFrontCenter("Cone")[0] == -2169 ? 
        getFrontCenter("Cube") : getFrontCenter("Cone");
    }

    public static double[] closestObject() {
        if(getFrontCenter("Cube")[1] != -2169 || getFrontCenter("Cone")[1] != -2169) {
            return table.getEntry("front-Cone-Center").getDoubleArray(arr)[1] < table.getEntry("front-Cone-Center").getDoubleArray(arr)[1] ? 
        getFrontCenter("Cone") : getFrontCenter("Cube");
        }
        else
            return arr;
    }

    public static String isThereObject() {
        if(getFrontCenter("Cone")[0] != -2169 && getFrontCenter("Cube")[0] != -2169)
            return "Both";
        else if(getFrontCenter("Cube")[0] != -2169) 
            return "Cube";
        else if(getFrontCenter("Cone")[0] != -2169)
            return "Cone";
        else
            return "None";
            
    }

    public static boolean isThereObjectPalm() {
        if(getPalmCenter("Cube") == arr || getPalmCenter("Cone") == arr) {
            return false;
        } else {
            return true;
        }

            
    }

    public static double frontApriltagYaw() {
        return table.getEntry("front-apriltag-Yaw").getDouble(-2169);
    }
    public static long frontApriltagId() {
        return table.getEntry("front-apriltag-Id").getInteger(-2169);
    }
    public static double frontApriltagX() {
        return table.getEntry("front-apriltag-X").getDouble(-2169);
    }
    public static double leftApriltagYaw() {
        return table.getEntry("Left-apriltag-Yaw").getDouble(-2169);
    }
    public static long leftApriltagId() {
        return table.getEntry("Left-apriltag-Id").getInteger(-2169);
    }
    public static double leftApriltagX() {
        return table.getEntry("Left-apriltag-X").getDouble(-2169);
    }
    public static double rightApriltagYaw() {
        return table.getEntry("Right-apriltag-Yaw").getDouble(-2169);
    }
    public static long rightApriltagId() {
        return table.getEntry("Right-apriltag-Id").getInteger(-2169);
    }
    public static double rightApriltagX() {
        return table.getEntry("Right-apriltag-X").getDouble(-2169);
    }
    public static double rightApriltagY() {
        return table.getEntry("Right-apriltag-Y").getDouble(-2169);
    }
    public static double leftApriltagY() {
        return table.getEntry("Left-apriltag-Y").getDouble(-2169);
    }
    public static double frontApriltagY() {
        return table.getEntry("Front-apriltag-Y").getDouble(-2169);
    }
    public static double[] leftApriltagCenter() {
        return table.getEntry("Left-apriltag-Center").getDoubleArray(arr);
    }
    public static double[] rightApriltagCenter() {
        return table.getEntry("Right-apriltag-Center").getDoubleArray(arr);
    }
    public static double[] frontApriltagCenter() {
        return table.getEntry("Front-apriltag-Center").getDoubleArray(arr);
    }

        
}
