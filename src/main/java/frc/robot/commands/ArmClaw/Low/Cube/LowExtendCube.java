package frc.robot.commands.ArmClaw.Low.Cube;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class LowExtendCube extends CommandBase {


    private Arm arm;
    private Claw claw;
    private double elevatorPosition = 293200;

    public LowExtendCube(Arm arm, Claw claw) {
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
    

    }
    @Override
	public void end(boolean interrupted) {
        if(!interrupted)
            claw.unGrab();

	}


    
    @Override
    public boolean isFinished() {
        return arm.getElevatorEncoder() > (elevatorPosition - 500);
    }
}