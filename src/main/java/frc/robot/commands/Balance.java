package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.Constants.ModuleConstants.*;

import frc.robot.Constants.DriveConstants;

public class Balance extends CommandBase {
    private SwerveSubsystem swerveSubsystem;

    private final ShuffleboardTab tab = Shuffleboard.getTab("Balance");
	private final GenericEntry pX = tab.addPersistent("P X", 0).getEntry();
	private final GenericEntry pY = tab.addPersistent("P Y", 0).getEntry();
	private final GenericEntry pTurn = tab.addPersistent("P Turn", 0).getEntry();

    private final PIDController pidTurn;
    private final PIDController pidX;
    private final PIDController pidY;
    private ChassisSpeeds chassisSpeeds;
    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;
    private boolean balanced;

    
    public Balance(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
        pidTurn = new PIDController(0.5, 0, 0);
        pidX = new PIDController(0.8, 0, 0);
        pidY = new PIDController(0.8, 0, 0);
        tab.addBoolean("Balanced?", () -> balanced);
    }


    @Override
    public void initialize() {
        pidTurn.setP(pTurn.getDouble(0));
		pidX.setP(pX.getDouble(0));
		pidY.setP(pY.getDouble(0));
        
    }

    @Override
    public void execute() {

        double roll = -NavX.getRoll();
        double pitch = -NavX.getPitch();

        double total = Math.sqrt(Math.pow(roll, 2) + Math.pow(pitch, 2));
        if(roll > 0 && pitch < 0)
            total = -total;

       xSpeed = pidX.calculate(total, 0);
       //xSpeed = -pidX.calculate(NavX.getPitch(), 0);
       //chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);
       chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

       if(pidX.atSetpoint() && pidY.atSetpoint()) {
        balanced = true;
       } else {
        balanced = false;
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