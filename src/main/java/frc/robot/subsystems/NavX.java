package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class NavX {
    private static AHRS gyro = new AHRS(SPI.Port.kMXP);
    private ShuffleboardTab tab = Shuffleboard.getTab("NavX");


    public NavX() {
        tab.addDouble("Yaw", () -> getYaw());
        tab.addDouble("Pitch", () -> getPitch());
        tab.addDouble("Roll", () -> getRoll());
        tab.addDouble("Angle", () -> getAngle());
    }

    public static double getX() {
        return gyro.getYaw();
    }

    public static double getY() {
        return gyro.getPitch();
    }

    public static double getRoll() {
        return gyro.getRoll();
    }
    
    public static double getPitch() {
        return gyro.getPitch();
    }

    public static double getYaw() {
        return gyro.getYaw();
    }

    /**Don't use!
     * Get pose from Swerve Subsytem instead
     */
    public static Pose2d getPose() {
        return new Pose2d(getX(), getY(), getRotation2d());
    }

    public static void resetPose() {
        gyro.resetDisplacement();
        gyro.reset();
    }


    public static Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public static void reset() {
        gyro.reset();
    }
    
    public static double getAngle() {
        return gyro.getAngle();
    }

    
}
