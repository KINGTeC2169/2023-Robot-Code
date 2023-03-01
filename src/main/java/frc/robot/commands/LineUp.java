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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

public class LineUp extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveSubsystem swerve;
    private final Claw claw;
    private final Arm arm;


    private final ProfiledPIDController pidX;
    private final ProfiledPIDController pidY;
    private final ProfiledPIDController pidRotate;
    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;
    private ChassisSpeeds chassisSpeeds;
    private boolean centered;
    private boolean finished;
    private boolean extended;
    private int scorePos;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public LineUp(SwerveSubsystem swerve, Arm arm, Claw claw) {
        this.swerve = swerve;
        this.arm = arm;
        this.claw = claw;
        this.scorePos = 2;
        SmartDashboard.putNumber("P-X", 0.0012);
        SmartDashboard.putNumber("P-Y", 1.535);
        SmartDashboard.putNumber("P-Rotate", 0.045);
        SmartDashboard.putNumber("D-X", 0.0012);

        addRequirements(claw);
        addRequirements(swerve);
        addRequirements(arm);
        pidX = new ProfiledPIDController(0.0012, 0, 0, new Constraints(1, 2));
        pidY = new ProfiledPIDController(1.535, 0, 0, new Constraints(1, 2));
        pidRotate = new ProfiledPIDController(0.05, 0, 0, new Constraints(1, 1));

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pidRotate.setTolerance(10);
        pidX.setTolerance(30);
        pidX.setP(SmartDashboard.getNumber("P-X", 0.0012));
        pidRotate.setP(SmartDashboard.getNumber("P-Rotate", 0.045));
        pidY.setP(SmartDashboard.getNumber("P-Y", 1.535));
        pidX.setD(SmartDashboard.getNumber("D-X", 0.0012));
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        xSpeed = 0;
        ySpeed = 0;
        turningSpeed = 0;
        
        if(!centered && NetworkTables.apriltagYaw() != -2169) {

            System.out.println(NetworkTables.apriltagCenter()[0]);
            turningSpeed = pidRotate.calculate(-NetworkTables.apriltagYaw(), 0) * .2;
            xSpeed = pidX.calculate(-NetworkTables.apriltagCenter()[0], 0);
            ySpeed = pidY.calculate(NetworkTables.apriltagY(), 1);
            
           

            switch(scorePos) {
                case 0: 
                
                break;
                case 1: 
                
                break;
                case 2:
                
                break;
                case 3:
                
                break;
                case 4:
                
                break;
                case 5:
                
                break;
                case 6:
                
                break;
                case 7:
                
                break;
                case 8:
                
                break;
            }
            
            if(pidRotate.atSetpoint() && pidX.atSetpoint() && pidY.atSetpoint()){
                claw.unGrab();
            }
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
