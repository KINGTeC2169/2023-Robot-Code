package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class TrainingWheels extends CommandBase {
    private Arm arm;
    private Claw claw;
    private double armAngle = 47;
    private double wristAngle = -15;
    private double twistAngle = 0;

    public TrainingWheels(Arm arm, Claw claw) {
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
