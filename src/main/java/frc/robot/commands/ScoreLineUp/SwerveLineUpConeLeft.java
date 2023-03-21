package frc.robot.commands.ScoreLineUp;

import static frc.robot.Constants.ModuleConstants.maxNeoRadPerSec;
import static frc.robot.Constants.ModuleConstants.maxSpeed;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDApriltags;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.NetworkTables;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveLineUpConeLeft extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, slider, rightX, rightY;
    private final Supplier<Boolean> sideButton, trigger, fieldButton, leftTop, leftBottom;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean isFieldOriented, isSlowMode;
    private final int controlMode;
    private double angle;
    private PIDController turnPID;
    private Constraints constraints = new Constraints(300, 500);
    private ProfiledPIDController turnPID2 = new ProfiledPIDController(0.01, 0, 0, constraints);
    private final double kPTurn = 0.009;
    private final PIDController pidX;
    private final PIDController pidY;
    private final PIDController pidRotate;
    
    public SwerveLineUpConeLeft(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, 
    Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Double> slider, Supplier<Boolean> sideButton,
    Supplier<Boolean> trigger, int balls) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.slider = slider;
        this.sideButton = sideButton;
        this.trigger = trigger;
        this.rightX = null;
        this.rightY = null;
        this.controlMode = 0;
        this.fieldButton = null;
        this.leftTop = null;
        this.leftBottom = null;
        
        //this.fieldOrientedFunction = fieldOrientedFunction;

        this.xLimiter = new SlewRateLimiter(2);
        this.yLimiter = new SlewRateLimiter(2);
        this.turningLimiter = new SlewRateLimiter(2);
        turnPID = new PIDController(kPTurn, 0, 0);
        turnPID.setTolerance(5);

        pidX = new PIDController(PIDApriltags.px, 0, 0);
        pidY = new PIDController(PIDApriltags.py, 0, 0);
        pidRotate = new PIDController(PIDApriltags.pr, 0, 0);
        pidRotate.setTolerance(PIDApriltags.rotateTol);
        pidX.setTolerance(PIDApriltags.xTol);
        pidY.setTolerance(PIDApriltags.yTol);

        
        addRequirements(swerveSubsystem);
    }
    public SwerveLineUpConeLeft(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
    Supplier<Double> turningSpdFunction, Supplier<Double> rightX, Supplier<Double> rightY, Supplier<Boolean> fieldButton, 
    Supplier<Boolean> leftTop,Supplier<Boolean> leftBottom) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.slider = null;
        this.sideButton = null;
        this.trigger = null;
        this.rightX = rightX;
        this.rightY = rightY;
        this.controlMode = 1;
        this.fieldButton = fieldButton;
        this.leftTop = leftTop;
        this.leftBottom = leftBottom;


        turnPID = new PIDController(kPTurn, 0, 0);
        turnPID.setTolerance(5);
        turnPID2.enableContinuousInput(0, 360);

        pidX = new PIDController(PIDApriltags.px, 0, 0);
        pidY = new PIDController(PIDApriltags.py, 0, 0);
        pidRotate = new PIDController(PIDApriltags.pr, 0, 0);
        pidRotate.setTolerance(PIDApriltags.rotateTol);
        pidX.setTolerance(PIDApriltags.xTol);
        pidY.setTolerance(PIDApriltags.yTol);

        this.xLimiter = new SlewRateLimiter(1.5);
        this.yLimiter = new SlewRateLimiter(1.5);
        this.turningLimiter = new SlewRateLimiter(1.5);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
        SmartDashboard.putBoolean("Field Oriented", isFieldOriented);
        SmartDashboard.putBoolean("Slow mode", isSlowMode);

        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = -turningSpdFunction.get();

        if(controlMode == 0) {
            isSlowMode = sideButton.get();

            if(slider.get() < -0.3)
                isFieldOriented = true;
            else if(slider.get() > 0.3) 
                isFieldOriented = false;
            


            // Deadband: unsure if necessary for our controllers
            xSpeed = Math.abs(xSpeed) > .03 ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > .03 ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > .02 ? turningSpeed : 0.0;
            
            xSpeed = Math.pow(xSpeed, 3);
            ySpeed = Math.pow(ySpeed, 3);
            turningSpeed = Math.pow(turningSpeed, 3);

            if(!trigger.get()) {
                xSpeed *= 0.5;
                ySpeed *= 0.5;
                turningSpeed *= 0.5;
            }

            if(isSlowMode) {
                xSpeed *= 0.4;
                ySpeed *= 0.4;
                turningSpeed *= 0.4;
            }
        }
        else if(controlMode == 1) {
            double x = rightX.get();
            double y = rightY.get();
            x = Math.abs(x) > .3 ? x : 0.0;
            y = Math.abs(y) > .3 ? y : 0.0;

            xSpeed = Math.abs(xSpeed) > .03 ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > .03 ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > .02 ? turningSpeed : 0.0;
            
            xSpeed = Math.pow(xSpeed, 3);
            ySpeed = Math.pow(ySpeed, 3);
            turningSpeed = Math.pow(turningSpeed, 3);

            if(x != 0 || y != 0) {
                //angle = Math.abs(Math.atan2(x, -y));
                angle = Math.atan2(x, -y);

                angle = Math.toDegrees(angle);
                angle = (angle + 360) % 360;
                
                SmartDashboard.putNumber("Control angle", angle);

               turningSpeed = turnPID2.calculate(swerveSubsystem.getHeading(), angle);
            }

            if(leftBottom.get()) {
                xSpeed *= swerveSubsystem.getSlowSpeed();
                ySpeed *= swerveSubsystem.getSlowSpeed();
                turningSpeed *= swerveSubsystem.getSlowSpeed();
            }
            else if(leftTop.get()) {
                xSpeed *= swerveSubsystem.getFastSpeed();
                ySpeed *= swerveSubsystem.getFastSpeed();
                turningSpeed *= swerveSubsystem.getFastSpeed();
            }
            else {
                xSpeed *= swerveSubsystem.getMediumSpeed();
                ySpeed *= swerveSubsystem.getMediumSpeed();
                turningSpeed *= swerveSubsystem.getMediumSpeed();
            }
            isFieldOriented = !fieldButton.get();
        }




        xSpeed = xLimiter.calculate(xSpeed) *  maxSpeed;
        ySpeed = yLimiter.calculate(ySpeed) *  maxSpeed;
        turningSpeed = turningLimiter.calculate(turningSpeed) * maxNeoRadPerSec;
        SmartDashboard.putNumber("Turn Speed", turningSpeed);

        ChassisSpeeds chassisSpeeds;

        if(NetworkTables.apriltagYaw() != -2169 && !(pidRotate.atSetpoint() && pidX.atSetpoint() && pidY.atSetpoint())) {


            xSpeed = xSpeed + pidX.calculate(-NetworkTables.apriltagX(), -.55);
            turningSpeed = turningSpeed + pidRotate.calculate(-NetworkTables.apriltagYaw(), -3);
      
        }
        
        //if(fieldOrientedFunction.get()) {
        if(isFieldOriented) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        }
        else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
    }



    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

  
    @Override
    public boolean isFinished() {
        return false;
    }
}
