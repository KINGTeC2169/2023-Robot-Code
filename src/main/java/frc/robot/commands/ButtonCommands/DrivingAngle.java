package frc.robot.commands.ButtonCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class DrivingAngle extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Claw claw;
	private final Arm arm;

	private double armAngle = 40;
    private double wristAngle = -115;
    private double elevatorPosition = 900;

    public DrivingAngle(Claw claw, Arm arm) {
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
		return true;
	}
}
