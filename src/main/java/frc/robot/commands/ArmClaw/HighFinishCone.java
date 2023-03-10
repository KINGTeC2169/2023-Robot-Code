package frc.robot.commands.ArmClaw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class HighFinishCone extends CommandBase {


    private Arm arm;
    private Claw claw;

    public HighFinishCone(Arm arm, Claw claw) {
        this.arm = arm;
        this.claw = claw;
        addRequirements(arm, claw);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        arm.setElevatorPosition(230560);
        arm.setArmAngle(37);
    

    }
    @Override
	public void end(boolean interrupted) {
        if(!interrupted)
            claw.unGrab();

	}


    
    @Override
    public boolean isFinished() {
        return arm.getElevatorEncoder() < 231000 && Math.abs(arm.getLiftAngle() - 37) < 2;
    }
}
