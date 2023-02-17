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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LineUp extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveSubsystem swerve;

    private final PIDController pidX;
    private final PIDController pidY;
    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;
    private ChassisSpeeds chassisSpeeds;
    private boolean centered;
    private double average;
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
        pidX = new PIDController(0.5, 0, 0);
        pidY = new PIDController(0.5, 0, 0);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if(!centered) {
            int divider = 2;
            double average = 0;
            if(CuboneManager.isConeInClaw()) {
                if(NetworkTables.leftApriltagX() == -1 || NetworkTables.rightApriltagX() == -1)
                    divider = 1;


                average = (Math.abs(NetworkTables.leftApriltagX()) + Math.abs(NetworkTables.rightApriltagX())) / divider;
            } else if(CuboneManager.isCubeInClaw()) {
                    
                //TODO: fix this, 0 is not the middle
                average = (Math.abs(NetworkTables.frontApriltagX()));
            }
            

            
            
            xSpeed = pidX.calculate(average, CuboneManager.isConeInClaw() ? Constants.Vision.apriltagOffset : 0);

            if(pidX.atSetpoint())
                centered = true;

            

        } else if(!wallBanged) {
            int divider = 2;
            if(NetworkTables.leftApriltagY() == -1 || NetworkTables.rightApriltagY() == -1)
                divider = 1;

            double average = (Math.abs(NetworkTables.leftApriltagY()) + Math.abs(NetworkTables.rightApriltagY())) / divider;
            
            ySpeed = pidY.calculate(average, 0);
            if(pidY.atSetpoint())
                wallBanged = true;
            
        } else {
            finished = true;
        }

        

        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);


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
//time.getUsClock()() > 1 && Math.abs(NetworkTable.getAngle()) < 5?
//return isAngle;
        return finished;
    }
}
