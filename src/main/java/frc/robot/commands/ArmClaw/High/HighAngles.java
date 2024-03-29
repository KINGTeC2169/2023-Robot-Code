package frc.robot.commands.ArmClaw.High;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class HighAngles extends CommandBase {


    private Arm arm;
    private Claw claw;
    private double armAngle = 40;
    private double wristAngle = 0;

    public HighAngles(Arm arm, Claw claw) {
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

    }
    
    @Override
    public boolean isFinished() {
        return Math.abs(arm.getLiftAngle() - armAngle) < 5 && Math.abs(claw.getWristEncoder() - wristAngle) < 10;
    }
}
