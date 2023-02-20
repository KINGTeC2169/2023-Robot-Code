package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnToPosition extends CommandBase {
    
    private final SwerveSubsystem swerveSubsystem;
    private double newAngle;
    private PIDController turnPID;

    public TurnToPosition(SwerveSubsystem swerveSubsystem, double newAngle) {
        this.swerveSubsystem = swerveSubsystem;
        this.newAngle = newAngle;
        turnPID = new PIDController(0.09, 0, 0);
        turnPID.setTolerance(5);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        newAngle = NavX.getAngle() + 90;
    }

    @Override
    public void execute() {
        
        ChassisSpeeds chassisSpeeds;
        
        //System.out.println(turningSpeed);
        chassisSpeeds = new ChassisSpeeds(0, 0, turnPID.calculate(NavX.getAngle(), newAngle));


        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
        System.out.println("Turning");
        SmartDashboard.putBoolean("At Setpoint", turnPID.atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        System.out.println("DONE!!!!!");
        return Math.abs(NavX.getAngle() - newAngle) < 10;
    }

}
