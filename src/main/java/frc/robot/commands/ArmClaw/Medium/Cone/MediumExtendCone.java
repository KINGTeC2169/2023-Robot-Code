package frc.robot.commands.ArmClaw.Medium.Cone;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class MediumExtendCone extends CommandBase {


   
    private Claw claw;
    private double wristPosition = -49;
    

    public MediumExtendCone(Claw claw) {
        
        this.claw = claw;
       
        addRequirements(claw);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        claw.setWristAngle(wristPosition);
    

    }
    @Override
	public void end(boolean interrupted) {
        if(!interrupted) 
            claw.unGrab();
	}


    
    @Override
    public boolean isFinished() {
        return Math.abs(claw.getWristEncoder() - wristPosition) < 10;
    }
}
