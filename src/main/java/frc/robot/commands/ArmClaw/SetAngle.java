package frc.robot.commands.ArmClaw;


import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SetAngle extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Claw claw;
	private final Arm arm;

    private double armAngle = 35;
    private double wristAngle = -115;
    private double elevatorPos = 90000 / 4;
	

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public SetAngle(Claw claw, Arm arm) {
		this.claw = claw;
		
		this.arm = arm;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(claw);
	
		addRequirements(arm);

		
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		//51931 elevator 23 angle
		arm.setArmAngle(armAngle);
        claw.setWristAngle(wristAngle);
        arm.setElevatorPosition(elevatorPos);

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
        return Math.abs(arm.getLiftAngle() - armAngle) < 5 && Math.abs(claw.getWristEncoder() - wristAngle) < 10 && Math.abs(arm.getElevatorEncoder() - elevatorPos) < 1000;
	}
}
