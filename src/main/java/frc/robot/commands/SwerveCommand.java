package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.Constants.ModuleConstants.*;

import frc.robot.Constants.DriveConstants;

public class SwerveCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    //private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    
    public SwerveCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, 
    Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        //this.fieldOrientedFunction = fieldOrientedFunction;

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
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // Deadband: unsure if necessary for our controllers
        xSpeed = Math.abs(xSpeed) > .05 ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > .05 ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > .02 ? turningSpeed : 0.0;
        

        xSpeed = xLimiter.calculate(xSpeed) *  maxSpeed;
        ySpeed = yLimiter.calculate(ySpeed) *  maxSpeed;
        turningSpeed = turningLimiter.calculate(turningSpeed) * maxNeoRadPerSec;

        System.out.println(xSpeed + "\t" + ySpeed + "\t" + turningSpeed);

        ChassisSpeeds chassisSpeeds;
        //if(fieldOrientedFunction.get()) {
        if(false) {
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
