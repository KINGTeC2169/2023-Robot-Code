package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.NetworkTables;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.Constants.ModuleConstants.*;

import frc.robot.Constants.DriveConstants;

public class ApriltagFollow extends CommandBase {
    private SwerveSubsystem swerveSubsystem;

    private final PIDController pidTurn;
    private final PIDController pidX;
    private final PIDController pidY;
    private ChassisSpeeds chassisSpeeds;
    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;
    private double target;

    public ApriltagFollow(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
        pidTurn = new PIDController(3.3, 0, 0);
        pidX = new PIDController(0.5, 0, 0);
        pidY = new PIDController(0.5, 0, 0);
    }

    @Override
    public void execute() {
        target = NetworkTables.frontApriltagX();
        if(target != -1) {
            turningSpeed = -pidTurn.calculate(target, 0);
            System.out.println(target);
            SmartDashboard.putNumber("turning speed", turningSpeed);
            //System.out.println(turningSpeed);
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);


            SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

            swerveSubsystem.setModuleStates(moduleStates);
        }
        
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
