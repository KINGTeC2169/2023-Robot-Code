package frc.robot.commands.ArmClaw.Medium.Cone;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class MediumExtendCone extends CommandBase {


    private Arm arm;
    private double elevatorPosition = 305000;
    

    public MediumExtendCone(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
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

	}


    
    @Override
    public boolean isFinished() {
        return arm.getElevatorEncoder() > (elevatorPosition - 500);
    }
}
