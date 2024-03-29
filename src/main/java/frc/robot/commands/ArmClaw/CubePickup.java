package frc.robot.commands.ArmClaw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class CubePickup extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Claw claw;
	private final Arm arm;

	
    private double elevatorPosition = 2478 / 4;
    private double winchPos = 14;
    private double wristAngle = -56;
    

    public CubePickup(Claw claw, Arm arm) {
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
		arm.setElevatorPosition(elevatorPosition);
        claw.setWristAngle(wristAngle);
        arm.setWinch(winchPos);
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
