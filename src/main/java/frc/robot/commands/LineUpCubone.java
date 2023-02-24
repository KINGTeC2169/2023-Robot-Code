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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

public class LineUpCubone extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveSubsystem swerve;

    private final ProfiledPIDController pidY;
    private final ProfiledPIDController pidRotate;
    private double ySpeed;
    private double turningSpeed;
    private ChassisSpeeds chassisSpeeds;
    private boolean rotated;
    private boolean cuboneBanged;
    private boolean finished;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public LineUpCubone(SwerveSubsystem swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
        pidY = new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(5, 10));
        pidRotate = new ProfiledPIDController(0.05, 0, 0, new TrapezoidProfile.Constraints(2, 4));

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pidRotate.setTolerance(4);
        
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ySpeed = 0;
        turningSpeed = 0;
        
        if(!rotated && NetworkTables.closestObject()[0] != -2169) {
            turningSpeed = MathUtil.clamp(pidRotate.calculate(NetworkTables.closestObject()[0], 320), -0.3, 0.3);
            System.out.println(turningSpeed);
            if(pidRotate.atSetpoint()){
                rotated = true;
            }
        } else if(!cuboneBanged) {
            ySpeed = .3;
            if(NetworkTables.isThereObjectPalm()){
                ySpeed = 0;
                cuboneBanged = true;
            }
        } else if(rotated && cuboneBanged){
            finished = true;
        }

        chassisSpeeds = new ChassisSpeeds(0, ySpeed, turningSpeed);


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
