package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTables {

    private static double[] arr = {-2169,-2169};
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    public NetworkTables() {}

    public static double[] getPalmCenter(String coneOrCube) {
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
            return Math.abs(table.getEntry("Front-Cone-Center").getDoubleArray(arr)[1]) < Math.abs(table.getEntry("Front-Cube-Center").getDoubleArray(arr)[1]) ? 
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
        return table.getEntry("left-apriltag-Yaw").getDouble(-2169);
    }
    public static long leftApriltagId() {
        return table.getEntry("left-apriltag-Id").getInteger(-2169);
    }
    public static double leftApriltagX() {
        return table.getEntry("left-apriltag-X").getDouble(-2169);
    }
    public static double rightApriltagYaw() {
        return table.getEntry("right-apriltag-Yaw").getDouble(-2169);
    }
    public static long rightApriltagId() {
        return table.getEntry("right-apriltag-Id").getInteger(-2169);
    }
    public static double rightApriltagX() {
        return table.getEntry("right-apriltag-X").getDouble(-2169);
    }
    public static double rightApriltagY() {
        return table.getEntry("right-apriltag-Y").getDouble(-2169);
    }
    public static double leftApriltagY() {
        return table.getEntry("left-apriltag-Y").getDouble(-2169);
    }
    public static double frontApriltagY() {
        return table.getEntry("front-apriltag-Y").getDouble(-2169);
    }
    public static double[] leftApriltagCenter() {
        return table.getEntry("left-apriltag-Center").getDoubleArray(arr);
    }
    public static double[] rightApriltagCenter() {
        return table.getEntry("right-apriltag-Center").getDoubleArray(arr);
    }
    public static double[] frontApriltagCenter() {
        return table.getEntry("front-apriltag-Center").getDoubleArray(arr);
    }


    //These are custom methods that combine all the cameras to get the most information possible

    

    public static String closestApriltag() {
        double closest = -2169;
        String closestString = null;
        if(frontApriltagYaw() != -2169 && Math.abs(closest) > Math.abs(frontApriltagYaw()) ) {
            closest = frontApriltagYaw();
            closestString = "front";
        }
        if(leftApriltagYaw() != -2169 && Math.abs(closest) > Math.abs(leftApriltagYaw()) ) {
            closest = leftApriltagYaw();
            closestString = "left";
        }
        if(rightApriltagYaw() != -2169 && Math.abs(closest) > Math.abs(rightApriltagYaw()) ) {
            closest = rightApriltagYaw();
            closestString = "right";
        }
        
        return closestString;
    }

    public static double apriltagId() {
        if(closestApriltag().equals(null)) {
            return table.getEntry("" + closestApriltag() + "-apriltag-Id").getDouble(-2169);
        }
        return -2169;
    }



    //TODO: FIX the 20 degree difference here
    //The Y value of the center might not work, but i dont think we use it so it should be good 
    public static double apriltagYaw() {
        double adder = 0;
        if(closestApriltag() != null) {
            
            if(closestApriltag().equals("front")) {
                adder = 0;
            } else if(closestApriltag().equals("left")) {
                adder = -30;
            } else if(closestApriltag().equals("right")) {
                adder = 30;
            }
            if(table.getEntry("" + closestApriltag() + "-apriltag-Yaw").getDouble(-2169) != -2169) {
                return table.getEntry("" + closestApriltag() + "-apriltag-Yaw").getDouble(-2169) + adder;
            }
        }
        return -2169;
    }

    public static double apriltagX() {
        return table.getEntry("" + closestApriltag() + "-apriltag-X").getDouble(-2169);
    }

    public static double[] apriltagCenter() {
        double adder = 0;
        if(closestApriltag() != null) {
            
            if(closestApriltag().equals("front")) {
                adder = -320;
            } else if(closestApriltag().equals("left")) {
                adder = -1120;
            } else if(closestApriltag().equals("right")) {
                adder = 320;
            }
            if(table.getEntry("" + closestApriltag() + "-apriltag-Center").getDoubleArray(arr) != arr) {
                double[] cringe = {table.getEntry("" + closestApriltag() + "-apriltag-Center").getDoubleArray(arr)[0] + adder, (table.getEntry("" + closestApriltag() + "-apriltag-Center").getDoubleArray(arr)[1])};
                return cringe;
            }
        }
        return arr;
    }

    
    public static double apriltagY() {
        return table.getEntry("" + closestApriltag() + "-apriltag-Y").getDouble(-2169);
    }
    
}
    
