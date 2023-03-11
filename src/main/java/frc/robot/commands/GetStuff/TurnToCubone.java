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
public class TurnToCubone extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	
	private final SwerveSubsystem swerve;
	

	private final ShuffleboardTab tab = Shuffleboard.getTab("GetCubone");
	
	private final GenericEntry pTurn = tab.addPersistent("P Turn", 0).getEntry();
	
	private final GenericEntry rotateTol = tab.addPersistent("Turn Tol", 0).getEntry();
	


	
	private double turningSpeed;
	private ChassisSpeeds chassisSpeeds;
	
	private final PIDController pidTurn;
	
	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public TurnToCubone(SwerveSubsystem swerve) {
		
		this.swerve = swerve;
		
		// Use addRequirements() here to declare subsystem dependencies.
	
		addRequirements(swerve);
		

		
		
		pidTurn = new PIDController(0.5, 0, 0);
		
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		
		pidTurn.setTolerance(rotateTol.getDouble(0));
		
		pidTurn.setP(pTurn.getDouble(0));
	
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

        if(NetworkTables.closestObject()[0] != 2169) {
            turningSpeed = pidTurn.calculate(NetworkTables.closestObject()[0], 0);
        }
		

		chassisSpeeds = new ChassisSpeeds(0, 0, turningSpeed);
			

		SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

		swerve.setModuleStates(moduleStates);


		//System.out.println("Current angle: " + currentAngle + "\tAngle: " + power + "\tPower: " + power);


	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
        swerve.stopModules();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return pidTurn.atSetpoint();
	}
}
