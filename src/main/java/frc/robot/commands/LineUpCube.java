package frc.robot.commands;


import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Motors;
import frc.robot.Constants.PIDApriltags;
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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

public class LineUpCube extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveSubsystem swerve;


    private final PIDController pidX;
    private final PIDController pidY;
    private final PIDController pidRotate;
    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;
    private ChassisSpeeds chassisSpeeds;
    private boolean centered;
    private boolean finished;
    private boolean extended;
    private int scorePos;
    private boolean x = false;
    private boolean y = false;
    private boolean turn = true;

    

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public LineUpCube(SwerveSubsystem swerve) {
        this.swerve = swerve;
        /*SmartDashboard.putNumber("P-X", 0.0016);
        SmartDashboard.putNumber("I-X", 0);
        SmartDashboard.putNumber("D-X", 0);
        SmartDashboard.putNumber("P-Rotate", .05);
        SmartDashboard.putNumber("I-Rotate", 0);
        SmartDashboard.putNumber("D-Rotate", 0);
        SmartDashboard.putNumber("P-Y", .7);
        SmartDashboard.putNumber("I-Y", 0);
        SmartDashboard.putNumber("D-Y", 0);*/
        
       
        
        
       

        addRequirements(swerve);
        pidX = new PIDController(PIDApriltags.px, 0, 0);
        pidY = new PIDController(PIDApriltags.py, 0, 0);
        pidRotate = new PIDController(PIDApriltags.pr, 0, 0);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        x = false;
        y = false;
        turn = true;
        centered = false;
        pidRotate.setTolerance(PIDApriltags.rotateTol);
        pidX.setTolerance(PIDApriltags.xTol);
        pidY.setTolerance(PIDApriltags.yTol);
        
        //pidX.setI(SmartDashboard.getNumber("I-X", 0));
        //pidX.setD(SmartDashboard.getNumber("D-X", 0));
        
        //pidRotate.setI(SmartDashboard.getNumber("I-Rotate", 0));
        //pidRotate.setD(SmartDashboard.getNumber("D-Rotate", 0));
        
        //pidY.setI(SmartDashboard.getNumber("I-Y", 0));
        //pidY.setD(SmartDashboard.getNumber("D-Y", 0));
        pidRotate.calculate(-NetworkTables.apriltagYaw(), 0);
        pidX.calculate(-NetworkTables.apriltagX(), 0);
        pidY.calculate(NetworkTables.apriltagY(), .56);
        
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        xSpeed = 0;
        ySpeed = 0;
        turningSpeed = 0;
        

        if(NetworkTables.apriltagYaw() != -2169 && !(pidRotate.atSetpoint() && pidX.atSetpoint() && pidY.atSetpoint())) {

            xSpeed = pidX.calculate(-NetworkTables.apriltagX(), 0);
            turningSpeed = pidRotate.calculate(-NetworkTables.apriltagYaw(), 0);
            if(pidX.atSetpoint() && pidRotate.atSetpoint()) {
                centered = true;
            }
            if(centered) {
                ySpeed = pidY.calculate(NetworkTables.apriltagY(), .58);
            }
    
        }
        

        
        chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);


        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        swerve.setModuleStates(moduleStates);



    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pidRotate.atSetpoint() && pidX.atSetpoint() && pidY.atSetpoint();
    }
}
