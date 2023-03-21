package frc.robot.commands.ButtonCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class PickUpAngle extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Claw claw;
	private final Arm arm;


	private double xSpeed;
	private double ySpeed;
	private double turningSpeed;
	private boolean gottem;
	private boolean itemCentered;
	private ChassisSpeeds chassisSpeeds;
	
	private final PIDController pidTurn;
	private final PIDController pidX;
	private final PIDController pidY;

    public PickUpAngle(Claw claw, Arm arm) {
        this.claw = claw;
        this.arm = arm;

        addRequirements(claw);
		addRequirements(arm);
        
        pidTurn = new PIDController(0.5, 0, 0);
        pidX = new PIDController(0.5, 0, 0);
        pidY = new PIDController(0.5, 0, 0);
    }
    
    @Override
	public void initialize() {
	}

    // Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
    }

    // Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}