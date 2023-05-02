package frc.robot.commands.ArmClaw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ResetArmClaw extends CommandBase {
    
    private Arm arm;
    private double elevatorPosition = 10000;
    private double armAngle = 47;

    public ResetArmClaw(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        arm.setElevatorPosition(elevatorPosition);
        arm.setArmAngle(armAngle);
    }
    @Override
	public void end(boolean interrupted) {
	}


    @Override
    public boolean isFinished() {
        return arm.getElevatorEncoder() < (elevatorPosition + 500) && Math.abs(arm.getLiftAngle() - armAngle) < 2;
    }
}

