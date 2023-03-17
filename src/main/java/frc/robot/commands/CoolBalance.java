package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SwerveSubsystem;

public class CoolBalance extends CommandBase {
    private SwerveSubsystem swerveSubsystem;

    private final PIDController pidX;
    private final PIDController pidY;
    private ChassisSpeeds chassisSpeeds;
    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;
    private boolean balanced;

    
    public CoolBalance(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
        pidX = new PIDController(0.04, 0, 0);
        pidY = new PIDController(0.04, 0, 0);
        
    }


    @Override
    public void initialize() {

        
    }

    @Override
    public void execute() {

        
        double roll = -NavX.getRoll();
        double pitch = -NavX.getPitch();

        if(pitch < -10) {
            double total = Math.sqrt(Math.pow(roll, 2) + Math.pow(pitch, 2));
            if(roll > 0 && pitch < 0)
                total = -total;

            xSpeed = pidX.calculate(total, 0);
        }
        else {
            xSpeed = 3;
        }

        
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
