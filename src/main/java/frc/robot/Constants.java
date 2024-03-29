// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Ports {
        public static final int clawGrippers = 16;
        public static final int winchMotor = 3;
        public static final int elevatorMotor = 14;
        public static final int wristMotor = 15;
        public static final int controller = 0;
        public static final int buttonBoard = 10;
        public static final int clawTwist = 9;
        public static final int colorSensorClaw = 1;

        public static final int armEncoder = 0;
        public static final int wristEncoder = 1;
        public static final int twistEncoder = 2;

        public static final int grabberOne = 8;
        public static final int grabberTwo = 10;

        //Swervedrive ports
        public static final int frontRightDrive = 8;
        public static final int frontRightTurn = 7;
        public static final int frontRightAbsolute = 11;
        public static final int frontLeftDrive = 6;
        public static final int frontLeftTurn = 5;
        public static final int frontLeftAbsolute = 13;
        public static final int backRightDrive = 2;
        public static final int backRightTurn = 20;
        public static final int backRightAbsolute = 12;
        public static final int backLeftDrive = 4;
        public static final int backLeftTurn = 3;
        public static final int backLeftAbsolute = 10;
    }

    public static final class Motors {
        public static final int TalonFXCPR = 2048;
        public static final int TalonSRXCPR = 8192;
    }

    public static final class ModuleConstants {
        //public static final double maxNeoSpeed = 3.68808;
        public static final double maxSpeed = 4.96824;
        public static final double maxNeoRadPerSec = 2 * 2 * Math.PI;
        public static final double wheelDiameter = 0.1016;//Units.inchesToMeters(4.0);
        public static final double driveGearRatio = 1 / 6.75;
        public static final double turnGearRatio = 1 / 12.8;
        public static final double driveEncoderToMeter = driveGearRatio * Math.PI * wheelDiameter;
        public static final double turnEncoderToRadian = turnGearRatio * 2 * Math.PI;
        public static final double driveEncoderRPMToMeterPerSec = driveEncoderToMeter / 60;
        public static final double turnEncoderRPMToRadPerSec = turnEncoderToRadian / 60;

        public static final double PTurn = 0.3;
        public static final double PDrive = 0.3;
    }
    public static final class PIDApriltags {
        public static final double px = 1.25;
        public static final double pr = 0.01;
        public static final double py = 1;
        public static final double xTol = 0.1;
        public static final double yTol = 0.1;
        public static final double rotateTol = 1;
    }

    public static final class DriveConstants {
        //These will need to be in meters
        public static final double rightLeftWheels = Units.inchesToMeters(21.5);
        public static final double frontBackWheels = Units.inchesToMeters(21.5);

        public static final double FRabsoluteOffset = -0.893;
        public static final double FLabsoluteOffset = -1.33609;
        public static final double BRabsoluteOffset = -0.7225;
        public static final double BLabsoluteOffset = -1.453;

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(frontBackWheels / 2, rightLeftWheels / 2),//Front-Left
                new Translation2d(frontBackWheels / 2, -rightLeftWheels / 2),//Front-Right
                new Translation2d(-frontBackWheels / 2, rightLeftWheels / 2),//Back-Left
                new Translation2d(-frontBackWheels / 2, -rightLeftWheels / 2));//Back-Right
    }

    public static final class Vision {
        public static final double apriltagOffset = 20;
    }
}
