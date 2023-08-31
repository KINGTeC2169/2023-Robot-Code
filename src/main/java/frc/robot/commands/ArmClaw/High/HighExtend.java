package frc.robot.commands.ArmClaw.High;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class HighExtend extends CommandBase {


    private Arm arm;
    private double elevatorPosition = 320000 / 4;
    

    public HighExtend(Arm arm) {
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
