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

	private double armAngle = 35;
    private double wristAngle = -115;
    private double twistAngle = 0;
    private double elevatorPosition = 90000;

    public PickUpAngle(Claw claw, Arm arm) {
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
		claw.setTwistAngle(twistAngle);
		claw.setWristAngle(wristAngle);
		claw.grab();
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