package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CuboneManager extends SubsystemBase {
    private static boolean coneInFront;
    private static boolean cubeInFront;

    private ShuffleboardTab tab = Shuffleboard.getTab("Cubone Manager");

    public CuboneManager() {}

    @Override
	public void periodic() {
		// This method will be called once per scheduler run
        tab.addBoolean("Cone in Front", () -> coneInFront);
		tab.addBoolean("Cube in Front", () -> cubeInFront);
        switch(NetworkTables.isThereObject()) {
            case "Cube": cubeInFront = true; 
            coneInFront = false;
            break;
            case "Cone": coneInFront = true;
            cubeInFront = false;
            break;
            case "None": coneInFront = false;
            cubeInFront = false;
            break;
            case "Both": cubeInFront = true;
            coneInFront = true;
            break;
        }
	}

    public static boolean isConeInbound() {
        return NetworkTables.getPalmCenter("Cone")[0] != -2169;
    }

    public static boolean isCubeInbound() {
        return NetworkTables.getPalmCenter("Cube")[0] != -2169;
    }

    public static boolean isConeInFront() {
        return coneInFront;
    }

    public static boolean isCubeInFront() {
        return cubeInFront;
    }
    
    public static boolean isSomethingInFront() {
        return cubeInFront || coneInFront;
    }
}
