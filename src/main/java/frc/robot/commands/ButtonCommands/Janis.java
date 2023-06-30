package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class Janis extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Claw claw;
	private final Arm arm;

	private double armAngle = 61;
    private double wristAngle = -71;
    private double elevatorPosition = 14680;//89600 / 4;

    public Janis(Claw claw, Arm arm) {
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
		
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return (arm.getLiftAngle() - armAngle) < .5;
	}
}