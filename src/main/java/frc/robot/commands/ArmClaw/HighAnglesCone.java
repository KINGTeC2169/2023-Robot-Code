package frc.robot.commands.ArmClaw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class HighAnglesCone extends CommandBase {


    private Arm arm;
    private Claw claw;

    public HighAnglesCone(Arm arm, Claw claw) {
        this.arm = arm;
        this.claw = claw;
        addRequirements(arm, claw);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        arm.setArmAngle(47);
        claw.setWristAngleSlow(-15);
        claw.setTwistAngle(0);

    }
    
    @Override
    public boolean isFinished() {
        return Math.abs(arm.getLiftAngle() - 47) < 5 && Math.abs(claw.getWristEncoder() + 15) < 10 && Math.abs(claw.getTwistEncoder()) < 4;
    }
}
