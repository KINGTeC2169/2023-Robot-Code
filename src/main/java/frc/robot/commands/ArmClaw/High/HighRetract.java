package frc.robot.commands.ArmClaw.High;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class HighRetract extends CommandBase {


    private Arm arm;
    private Claw claw;
    private double elevatorPosition = 1000;
    private double armAngle = 37;

    public HighRetract(Arm arm, Claw claw) {
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
        

	}


    
    @Override
    public boolean isFinished() {
        return arm.getElevatorEncoder() < (elevatorPosition + 500);
    }
}
