package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.Ports;
import static frc.robot.Constants.ModuleConstants.*;


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
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kinematics, getRotation2d(), getModulePositions());
    public Field2d field = new Field2d();

    private ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
    private GenericEntry maxSpeed = tab.add("Max Speed", 1.0).getEntry();


    public SwerveSubsystem() {
        
        SmartDashboard.putData("Field", field);
        tab.addDouble("Robot Heading", () -> getHeading());

        tab.addDouble("Front Left", () -> frontLeft.getDriveCurrent());
        tab.addDouble("Front Right", () -> frontRight.getDriveCurrent());
        tab.addDouble("Back Left", () -> backLeft.getDriveCurrent());
        tab.addDouble("Back Right", () -> backRight.getDriveCurrent());
        
        tab.addDouble("Abs Front Left", () -> frontLeft.getAbsoluteTurnPosition());
        tab.addDouble("Abs Front Right", () -> frontRight.getAbsoluteTurnPosition());
        tab.addDouble("Abs Back Left", () -> backLeft.getAbsoluteTurnPosition());
        tab.addDouble("Abs Back Right", () -> backRight.getAbsoluteTurnPosition());


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
        System.out.println(maxSpeed.getDouble(1.0));

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
