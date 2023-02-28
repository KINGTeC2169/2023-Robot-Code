package frc.robot.commands;


import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Motors;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CuboneManager;
import frc.robot.subsystems.NetworkTables;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

public class LineUp extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveSubsystem swerve;


    private final PIDController pidX;
    private final PIDController pidY;
    private final PIDController pidRotate;
    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;
    private ChassisSpeeds chassisSpeeds;
    private boolean rotated;
    private boolean centered;
    private boolean wallBanged;
    private boolean finished;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public LineUp(SwerveSubsystem swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
        pidX = new PIDController(0.003, 0, 0);
        pidY = new PIDController(0.03, 0, 0);
        pidRotate = new PIDController(0.2, 0, 0);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pidRotate.setTolerance(10);
        pidX.setTolerance(30);
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        xSpeed = 0;
        ySpeed = 0;
        turningSpeed = 0;
        
        if(!centered && NetworkTables.frontApriltagCenter()[0] != -2169) {

            turningSpeed = pidRotate.calculate(-NetworkTables.apriltagYaw(), 0) * .2;
            
            xSpeed = pidX.calculate(-NetworkTables.frontApriltagCenter()[0], -320);
            
            //ySpeed = pidY.calculate(NetworkTables.frontApriltagY() * 39, 0);
            

        } else if(centered){
            finished = true;
        }
        chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);


        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        swerve.setModuleStates(moduleStates);



    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
}
