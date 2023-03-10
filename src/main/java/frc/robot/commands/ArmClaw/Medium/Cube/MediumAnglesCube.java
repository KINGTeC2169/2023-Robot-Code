package frc.robot.commands.ArmClaw.Medium.Cube;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class MediumAnglesCube extends CommandBase {


    private Arm arm;
    private Claw claw;
    private double armAngle = 45;
    private double wristAngle = -17;
    private double twistAngle = 0;

    public MediumAnglesCube(Arm arm, Claw claw) {
        this.arm = arm;
        this.claw = claw;
        addRequirements(arm, claw);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        arm.setArmAngle(armAngle);
        claw.setWristAngleSlow(wristAngle);
        claw.setTwistAngle(twistAngle);

    }
    
    @Override
    public boolean isFinished() {
        return Math.abs(arm.getLiftAngle() - armAngle) < 5 && Math.abs(claw.getWristEncoder() - wristAngle) < 10 && Math.abs(claw.getTwistEncoder() - twistAngle) < 4;
    }
}
