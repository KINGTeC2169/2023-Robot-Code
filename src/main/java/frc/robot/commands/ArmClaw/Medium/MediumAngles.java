package frc.robot.commands.ArmClaw.Medium;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class MediumAngles extends CommandBase {


    private Arm arm;
    private Claw claw;
    private double armAngle = 47;
    private double wristAngle = -15;
    private double twistAngle = 0;
    private double elevatorPosition = 1000;

    public MediumAngles(Arm arm, Claw claw) {
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
        arm.setElevatorPosition(elevatorPosition);
        claw.setWristAngleSlow(wristAngle);
        claw.setTwistAngle(twistAngle);

    }
    
    @Override
    public boolean isFinished() {
        return Math.abs(arm.getLiftAngle() - armAngle) < 5 && Math.abs(claw.getWristEncoder() - wristAngle) < 10 && Math.abs(claw.getTwistEncoder() - twistAngle) < 4 && arm.getElevatorEncoder() < (elevatorPosition + 500);
    }
}