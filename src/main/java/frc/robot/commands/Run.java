package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SwerveSubsystem;

public class Run extends CommandBase {
    private SwerveSubsystem swerveSubsystem;
    private Arm arm;
    private Claw claw;

    private ChassisSpeeds chassisSpeeds;
    private double xSpeed;
    private double ySpeed;
    
    public Run(SwerveSubsystem swerveSubsystem, Arm arm, Claw claw) {
        this.swerveSubsystem = swerveSubsystem;
        this.arm = arm;
        this.claw = claw;
        addRequirements(swerveSubsystem, arm, claw);
    }


    @Override
    public void initialize() {
        arm.setWinch(78);
        claw.setWristAngle(-105);
    }

    @Override
    public void execute() {

        xSpeed = 2;
       //xSpeed = -pidX.calculate(NavX.getPitch(), 0);
       //chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);
       chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, 0, swerveSubsystem.getRotation2d());

       

        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
    }



    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

  
    @Override
    public boolean isFinished() {
        return NavX.getPitch() > 9;
    }

}
