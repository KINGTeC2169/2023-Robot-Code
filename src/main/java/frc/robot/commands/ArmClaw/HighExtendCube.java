package frc.robot.commands.ArmClaw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class HighExtendCube extends CommandBase {


    private Arm arm;
    private Claw claw;
    private boolean imDoneMate;

    public HighExtendCube(Arm arm, Claw claw) {
        this.arm = arm;
        this.claw = claw;
        addRequirements(arm, claw);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        arm.setElevatorPosition(293200);
    

    }
    @Override
	public void end(boolean interrupted) {
        if(!interrupted)
            claw.unGrab();

	}


    
    @Override
    public boolean isFinished() {
        return arm.getElevatorEncoder() > 290000;
    }
}
