package frc.robot.commands.ArmClaw.Low;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class LowFinishCone extends CommandBase {


    private Arm arm;
    private Claw claw;
    private double elevatorPosition = 230560;
    private double armAngle = 37;

    public LowFinishCone(Arm arm, Claw claw) {
        this.arm = arm;
        this.claw = claw;
        addRequirements(arm, claw);
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
        if(!interrupted)
            claw.unGrab();

	}


    
    @Override
    public boolean isFinished() {
        return arm.getElevatorEncoder() < (elevatorPosition + 500) && Math.abs(arm.getLiftAngle() - armAngle) < 2;
    }
}
