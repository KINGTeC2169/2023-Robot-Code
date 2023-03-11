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
public class Persue extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Claw claw;
	private final SwerveSubsystem swerve;

    private double armAngle = 23;
    private double wristAngle = -17;
    private double twistAngle = 0;
    private double elevatorPos = 51931;
    private ChassisSpeeds chassisSpeeds;
	

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public Persue(Claw claw, SwerveSubsystem swerve) {
		this.claw = claw;
        this.swerve = swerve;
		
		
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(claw);
        addRequirements(swerve);
	
		

		
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {




        chassisSpeeds = new ChassisSpeeds(.2, 0, 0);
			

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
        return NetworkTables.isThereObjectPalm();
	}
}
