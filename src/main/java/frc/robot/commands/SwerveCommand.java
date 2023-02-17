package frc.robot.commands;

import static frc.robot.Constants.ModuleConstants.maxNeoRadPerSec;
import static frc.robot.Constants.ModuleConstants.maxSpeed;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, slider, direction;
    private final Supplier<Boolean> sideButton, trigger;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean isFieldOriented, isSlowMode;
    
    public SwerveCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, 
    Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Double> slider, Supplier<Boolean> sideButton,
    Supplier<Boolean> trigger) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.slider = slider;
        this.sideButton = sideButton;
        this.trigger = trigger;
        this.direction = null;
        //this.fieldOrientedFunction = fieldOrientedFunction;

        this.xLimiter = new SlewRateLimiter(5);
        this.yLimiter = new SlewRateLimiter(5);
        this.turningLimiter = new SlewRateLimiter(5);
        addRequirements(swerveSubsystem);
    }
    public SwerveCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
    Supplier<Double> turningSpdFunction, Supplier<Double> direction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.slider = null;
        this.sideButton = null;
        this.trigger = null;
        this.direction = direction;

        this.xLimiter = new SlewRateLimiter(5);
        this.yLimiter = new SlewRateLimiter(5);
        this.turningLimiter = new SlewRateLimiter(5);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Field Oriented", isFieldOriented);
        SmartDashboard.putBoolean("Slow mode", isSlowMode);
        isSlowMode = sideButton.get();

        if(slider.get() < -0.3)
            isFieldOriented = true;
        else if(slider.get() > 0.3) 
            isFieldOriented = false;
        
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // Deadband: unsure if necessary for our controllers
        xSpeed = Math.abs(xSpeed) > .03 ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > .03 ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > .02 ? turningSpeed : 0.0;
        
        //xSpeed = xSpeed >= 0 ? xSpeed * xSpeed : xSpeed * -xSpeed;
        //ySpeed = ySpeed >= 0 ? ySpeed * ySpeed : ySpeed * -ySpeed;
        //turningSpeed = turningSpeed >= 0 ? turningSpeed * turningSpeed : turningSpeed * -turningSpeed;
        xSpeed = Math.pow(xSpeed, 3);
        ySpeed = Math.pow(ySpeed, 3);
        turningSpeed = Math.pow(turningSpeed, 3);

        if(!trigger.get()) {
            xSpeed *= 0.5;
            ySpeed *= 0.5;
            turningSpeed *= 0.5;
        }

        if(isSlowMode) {
            xSpeed *= 0.2;
            ySpeed *= 0.2;
            turningSpeed *= 0.2;
        }


        xSpeed = xLimiter.calculate(xSpeed) *  maxSpeed;
        ySpeed = yLimiter.calculate(ySpeed) *  maxSpeed;
        turningSpeed = turningLimiter.calculate(turningSpeed) * maxNeoRadPerSec;

        System.out.println(xSpeed + "\t" + ySpeed + "\t" + turningSpeed);

        ChassisSpeeds chassisSpeeds;
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
