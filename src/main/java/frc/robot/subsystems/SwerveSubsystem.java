package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.Ports;
import static frc.robot.Constants.ModuleConstants.*;

import java.util.Map;


public class SwerveSubsystem extends SubsystemBase {

    //Creates an instance of SwerveModule for each module on the robot
    private SwerveModule frontLeft = new SwerveModule(
    Ports.frontLeftDrive,
    Ports.frontLeftTurn, 
    true, false,
    Ports.frontLeftAbsolute,
    DriveConstants.FLabsoluteOffset,
    false);

    private SwerveModule frontRight = new SwerveModule(
    Ports.frontRightDrive,
    Ports.frontRightTurn, 
    false, false,
    Ports.frontRightAbsolute,
    DriveConstants.FRabsoluteOffset,
    false);

    private SwerveModule backLeft = new SwerveModule(
    Ports.backLeftDrive,
    Ports.backLeftTurn, 
    true, false,
    Ports.backLeftAbsolute,
    DriveConstants.BLabsoluteOffset,
    false);

    private SwerveModule backRight = new SwerveModule(
    Ports.backRightDrive,
    Ports.backRightTurn, 
    false, false,
    Ports.backRightAbsolute,
    DriveConstants.BRabsoluteOffset,
    false);

    public SwerveDriveKinematics kinematics = DriveConstants.DRIVE_KINEMATICS;
    private final SwerveDriveOdometry odometer;
    public Field2d field = new Field2d();

    private ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
    private GenericEntry fastSpeed = tab.add("Fast Speed", 1.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("Min", 0)).withPosition(6, 3).getEntry();
    private GenericEntry mediumSpeed = tab.add("Medium Speed", 0.5).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("Min", 0)).withPosition(4, 3).getEntry();
    private GenericEntry slowSpeed = tab.add("Slow Speed", 0.2).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("Min", 0)).withPosition(2, 3).getEntry();

    public SwerveSubsystem() {
        odometer = new SwerveDriveOdometry(kinematics, getRotation2d(), getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)));
        
        //SmartDashboard.putData("Field", field);
        
        ShuffleboardLayout driveCurrents = tab.getLayout("Drive Currents", BuiltInLayouts.kGrid).withSize(2, 2).withProperties(Map.of("Number of rows", 2)).withPosition(0, 0);
        ShuffleboardLayout turnCurrents = tab.getLayout("Turn Currents", BuiltInLayouts.kGrid).withSize(2, 2).withProperties(Map.of("Number of rows", 2)).withPosition(0, 2);

        driveCurrents.addDouble("Front Left", () -> frontLeft.getDriveCurrent()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Orientation", "VERTICAL"));
        driveCurrents.addDouble("Front Right", () -> frontRight.getDriveCurrent()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Orientation", "VERTICAL"));
        driveCurrents.addDouble("Back Left", () -> backLeft.getDriveCurrent()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Orientation", "VERTICAL"));
        driveCurrents.addDouble("Back Right", () -> backRight.getDriveCurrent()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Orientation", "VERTICAL"));

        turnCurrents.addDouble("Front Left", () -> frontLeft.getTurnCurrent()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Orientation", "VERTICAL"));
        turnCurrents.addDouble("Front Right", () -> frontRight.getTurnCurrent()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Orientation", "VERTICAL"));
        turnCurrents.addDouble("Back Left", () -> backLeft.getTurnCurrent()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Orientation", "VERTICAL"));
        turnCurrents.addDouble("Back Right", () -> backRight.getTurnCurrent()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Orientation", "VERTICAL"));


        //tab.add("Test", "Balls").withWidget(BuiltInWidgets.kComboBoxChooser);

        tab.addDouble("Robot Heading", () -> getHeading()).withWidget(BuiltInWidgets.kGyro).withSize(3, 3).withPosition(7, 0);

        tab.addDouble("Rotation 2D", () -> getRotation2d().getDegrees());

        //tab.addDouble("Abs Front Left", () -> frontLeft.getAbsoluteTurnPosition());
        //tab.addDouble("Abs Front Right", () -> frontRight.getAbsoluteTurnPosition());
        //tab.addDouble("Abs Back Left", () -> backLeft.getAbsoluteTurnPosition());
        //tab.addDouble("Abs Back Right", () -> backRight.getAbsoluteTurnPosition());
        tab.addDouble("X", () -> odometer.getPoseMeters().getX()).withPosition(8, 3);
        tab.addDouble("Y", () -> odometer.getPoseMeters().getY()).withPosition(9, 3);

        tab.add(field).withPosition(2, 0).withSize(5, 3);

        //Creates a new thread, which sleeps and then zeros out the gyro
        //Uses a new thread so that it doesn't pause all other code running
        new Thread(() -> {
            try {
                Thread.sleep(5000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public Field2d getField() {
        return field;
    }

    public double getFastSpeed() {
        return fastSpeed.getDouble(1.0);
    }

    public double getMediumSpeed() {
        return mediumSpeed.getDouble(0.5);
    }

    public double getSlowSpeed() {
        return slowSpeed.getDouble(0.2);
    }
    
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
        frontLeft.getModulePosition(),
        frontRight.getModulePosition(),
        backLeft.getModulePosition(),
        backRight.getModulePosition()
        };
        
    }

    public void zeroHeading() {
        System.out.println("Zeroing gyro \n.\n.\n.\n.\n.\n.\n.");
        NavX.reset();
    }

    public double getHeading() {
        //return Math.IEEEremainder(NavX.getAngle(), 360);
        return NavX.getAngle() % 360;
    }

    public Rotation2d getRotation2d() {
        return NavX.getRotation2d();
        //return Rotation2d.fromDegrees(-getHeading());
    }
    

    public void resetPose(Pose2d pose) {
    }

    public Pose2d getPose() {
        //return NavX.getPose();
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }


    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backRight.resetEncoders();
        backLeft.resetEncoders();
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());

        //field.setRobotPose(odometer.getPoseMeters());
        field.setRobotPose(odometer.getPoseMeters().getX(), odometer.getPoseMeters().getY(), odometer.getPoseMeters().getRotation());

        SmartDashboard.putNumber("X", odometer.getPoseMeters().getX());
        SmartDashboard.putNumber("Y", odometer.getPoseMeters().getY());
        SmartDashboard.putNumber("Pose angle", odometer.getPoseMeters().getRotation().getDegrees());
        
        //SmartDashboard.putData("Field", field);
    
        /*
        SmartDashboard.putNumber("Front Left", frontLeft.getTurnPosition());
        SmartDashboard.putNumber("Front Right", frontRight.getTurnPosition());
        SmartDashboard.putNumber("Back Left", backLeft.getTurnPosition());
        SmartDashboard.putNumber("Back Right", backRight.getTurnPosition());
        SmartDashboard.putNumber("Wanted Speed", backRight.getWantedSpeed());
        SmartDashboard.putNumber("Back Right speed", backRight.getDriveVelocity());
        SmartDashboard.putNumber("Back Left speed", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("Front Right speed", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("Front left speed", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("Error", backLeft.getError());
        SmartDashboard.putNumber("Error", Math.abs(backRight.getWantedSpeed() - backRight.getDrivePosition()));
        */
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

     /**Takes an array of SwerveModuleStates and sets each SwerveModule to its respective state */
     public void setModuleStates(SwerveModuleState[] states) {

        SwerveDriveKinematics.desaturateWheelSpeeds(states, ModuleConstants.maxSpeed);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);

    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()};
    }

    /**Puts wheels in 'X' position and sets driving to a velocity-PID loop set at 0m/s */
    public void setActiveStop() {
        System.out.println("1\n1\n1\n1\n1\n1\n1\n1");
        frontLeft.activeStop(-1);
        frontRight.activeStop(1);
        backLeft.activeStop(1);
        backRight.activeStop(-1);
    }

}
