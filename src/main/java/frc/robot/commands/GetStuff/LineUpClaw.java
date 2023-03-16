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
public class LineUpClaw extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Claw claw;
	

	private final ShuffleboardTab tab = Shuffleboard.getTab("GetCubone");
	

	
	
	

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public LineUpClaw(Claw claw) {
		this.claw = claw;
		
	
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(claw);
	
		

	
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
        
		
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(CuboneManager.isConeInbound()){
			claw.setTwistAngle(NetworkTables.getPalmAngle("Cone"));
		}

        
		


	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
        
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		//return Math.abs(claw.getTwistEncoder() - NetworkTables.getPalmAngle("Cone")) < 10;
		return true;
	}
}
