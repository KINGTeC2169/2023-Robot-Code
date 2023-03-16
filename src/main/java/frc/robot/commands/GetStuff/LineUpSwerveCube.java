package frc.robot.commands.GetStuff;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CuboneManager;
import frc.robot.subsystems.NetworkTables;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class LineUpSwerveCube extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Claw claw;
	private final SwerveSubsystem swerve;
	

	private final ShuffleboardTab tab = Shuffleboard.getTab("GetCubone");
	
	private final GenericEntry pX = tab.addPersistent("P X", 0).getEntry();
	private final GenericEntry pY = tab.addPersistent("P Y", 0).getEntry();
	
	private final GenericEntry yTol = tab.addPersistent("Y Tol", 0).getEntry();

	private final GenericEntry xTol = tab.addPersistent("X Tol", 0).getEntry();
	


	private double xSpeed;
	private double ySpeed;
	private boolean itemCentered;
	private ChassisSpeeds chassisSpeeds;
	private boolean centered;
	private double angle;
	
	
	private final PIDController pidX;
	private final PIDController pidY;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public LineUpSwerveCube(Claw claw, SwerveSubsystem swerve) {
		this.claw = claw;
		this.swerve = swerve;
	
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(claw);
		addRequirements(swerve);
		

		
	
		pidX = new PIDController(0.5, 0, 0);
		pidY = new PIDController(0.5, 0, 0);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
        itemCentered = false;
		centered = false;
		claw.resetTwistEncoder();

		pidX.setTolerance(10);
		pidY.setTolerance(10);

		
		pidX.setP(-0.003);
		pidY.setP(-0.003);
		claw.setTwistAngle(0);
		pidX.calculate(NetworkTables.getPalmCenter("Cone")[0], 320);
        pidY.calculate(NetworkTables.getPalmCenter("Cone")[1], 240);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		xSpeed = 0;
		ySpeed = 0;

        if(CuboneManager.isSomethingInBound()) {

			
            
				if(CuboneManager.isCubeInbound()) {
					xSpeed = pidX.calculate(NetworkTables.getPalmCenter("Cube")[0], 320);
            		ySpeed = pidY.calculate(NetworkTables.getPalmCenter("Cube")[1], 260);
				}
			
			
            
        }
        
		

		chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, 0);
			

		SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

		swerve.setModuleStates(moduleStates);


		


	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
        swerve.fullStop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return pidX.atSetpoint() && pidY.atSetpoint();
	}
}
