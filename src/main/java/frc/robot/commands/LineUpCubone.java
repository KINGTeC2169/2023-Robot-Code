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

    private final PIDController pidY;
    private final PIDController pidRotate;
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
        pidY = new PIDController(0.05, 0, 0);
        pidRotate = new PIDController(0.03, 0, 0);

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
        
        if(!NetworkTables.isThereObjectPalm() && NetworkTables.getFrontCenter()[0] != -2169) {

        
            turningSpeed = pidRotate.calculate(-NetworkTables.getFrontCenter()[0], -320) * .2;
            System.out.print(NetworkTables.getFrontCenter()[0]);
            //if(NetworkTables.apriltagCenter()[0] -240 < 10)
                //ySpeed = pidY.calculate(NetworkTables.getFrontCenter()[1], 240);
            

        } else if(NetworkTables.isThereObjectPalm()){
            finished = true;
        }

        chassisSpeeds = new ChassisSpeeds(ySpeed, 0, turningSpeed);


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
