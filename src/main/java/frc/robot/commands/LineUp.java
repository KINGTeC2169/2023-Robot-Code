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
    public LineUp(SwerveSubsystem swerve) {
        this.swerve = swerve;
        SmartDashboard.putNumber("P-X", 0.012);
        SmartDashboard.putNumber("I-X", 0);
        SmartDashboard.putNumber("D-X", 0.00012);
        SmartDashboard.putNumber("P-Rotate", 0.14);
        SmartDashboard.putNumber("I-Rotate", 0);
        SmartDashboard.putNumber("D-Rotate", 0.0014);
        SmartDashboard.putNumber("P-Y", 1.535);
        SmartDashboard.putNumber("I-Y", 0);
        SmartDashboard.putNumber("D-Y", 0);
       
        
        
       

        addRequirements(swerve);
        pidX = new ProfiledPIDController(0.0012, 0, 0, new Constraints(1, 2));
        pidY = new ProfiledPIDController(1.535, 0, 0, new Constraints(1, 2));
        pidRotate = new ProfiledPIDController(0.05, 0, 0, new Constraints(1, 1));

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //pidRotate.setTolerance(5);
        //pidX.setTolerance(10);
        //pidY.setTolerance(10);
        pidX.setP(SmartDashboard.getNumber("P-X", 0.012));
        pidX.setI(SmartDashboard.getNumber("I-X", 0));
        pidX.setD(SmartDashboard.getNumber("D-X", 0.00012));
        pidRotate.setP(SmartDashboard.getNumber("P-Rotate", 0.14));
        pidRotate.setI(SmartDashboard.getNumber("I-Rotate", 0));
        pidRotate.setD(SmartDashboard.getNumber("D-Rotate", 0.0014));
        pidY.setP(SmartDashboard.getNumber("P-Y", 1.535));
        pidY.setI(SmartDashboard.getNumber("I-Y", 0));
        pidY.setD(SmartDashboard.getNumber("D-Y", 0));
        
        
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
            
            
            if(pidRotate.atSetpoint() && pidX.atSetpoint() && pidY.atSetpoint()){
                end(false);
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
