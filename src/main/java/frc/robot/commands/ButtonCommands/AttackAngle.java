package frc.robot.commands.ButtonCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class AttackAngle extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Claw claw;
	private final Arm arm;

	private double armAngle = 22;
    private double wristAngle = -115;
    private double elevatorPosition = 90000;

    public AttackAngle(Claw claw, Arm arm) {
        this.claw = claw;
        this.arm = arm;

        addRequirements(claw);
		addRequirements(arm);
    }
    
    @Override
	public void initialize() {
	}

    // Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		arm.setArmAngle(armAngle);
		arm.setElevatorPosition(elevatorPosition);
		claw.setWristAngle(wristAngle);
    }

    // Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		if(!interrupted) {
			claw.grab();
		}
		
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return (arm.getLiftAngle() - armAngle) < .5;
	}
}