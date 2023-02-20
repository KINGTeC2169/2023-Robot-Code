package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CuboneManager extends SubsystemBase {
    private static boolean coneInClaw;
    private static boolean cubeInClaw;
    private static boolean coneInFront;
    private static boolean cubeInFront;

    public CuboneManager() {}

    @Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putBoolean("Cone in Claw", coneInClaw);
		SmartDashboard.putBoolean("Cube in Claw", cubeInClaw);
        SmartDashboard.putBoolean("Cone in Front", coneInFront);
		SmartDashboard.putBoolean("Cube in Front", cubeInFront);
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

    public static boolean isConeInClaw() {
        return coneInClaw;
    }

    public static void setConeInClaw(boolean coneInClaw) {
        CuboneManager.coneInClaw = coneInClaw;
    }

    public static boolean isCubeInClaw() {
        return cubeInClaw;
    }

    public static void setCubeInClaw(boolean cubeInClaw) {
        CuboneManager.cubeInClaw = cubeInClaw;
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

    public static boolean isSomethingInClaw() {
        return cubeInClaw || coneInClaw;
    }
}
