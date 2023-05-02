package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.Constants.DriveConstants;

public class Balance extends CommandBase {
    private SwerveSubsystem swerveSubsystem;

    private final PIDController pidTurn;
    private final PIDController pidX;
    private ChassisSpeeds chassisSpeeds;
    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;

    
    public Balance(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
        pidTurn = new PIDController(0.5, 0, 0);
        pidX = new PIDController(0.1, 0.0, 0.0001);
        
    }


    @Override
    public void initialize() {
        pidTurn.setP(0);
		pidX.setP(0.05);
        pidX.setTolerance(1);
    }

    @Override
    public void execute() {

        double roll = -NavX.getRoll();
        double pitch = -NavX.getPitch();

        double total = Math.sqrt(Math.pow(roll, 2) + Math.pow(pitch, 2));
        if(roll > 0 && pitch < 0)
            total = -total;

       xSpeed = pidX.calculate(pitch, 0);
      // if (DriverStation.getMatchTime() < 0.1) {
       // xSpeed = 0;
        //ySpeed = 0.1;
      // }
       //xSpeed = -pidX.calculate(NavX.getPitch(), 0);
       //chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);

   
       chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());



        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
    }



    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

  
    @Override
    public boolean isFinished() {
        return NavX.getPitch() < -7;
    }
}