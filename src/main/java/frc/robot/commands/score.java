package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Motors;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CuboneManager;
import frc.robot.subsystems.NetworkTables;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Score extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Claw claw;
    private final SwerveSubsystem swerve;
    private final Arm arm;


    private final PIDController pidTurn;
    private final PIDController pidX;
    private final PIDController pidY;
    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;
    private double wristTurn;
    private double angle;
    private boolean isAngle;
    private boolean readingAngle;
    private boolean haveCone;
    private boolean haveCube;
    private ChassisSpeeds chassisSpeeds;
    private boolean centered;
    private double average;
    private boolean wallBanged;
    private Supplier<Integer> scorePositionSupplier;
    private int scorePosition;
    private boolean reachedArmAngle;
    private boolean extendedArm;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public Score(Claw claw, SwerveSubsystem swerve, Arm arm, Supplier<Integer> scorePositionSupplier) {
        this.claw = claw;
        this.swerve = swerve;
        this.arm = arm;
        this.scorePositionSupplier = scorePositionSupplier;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(claw);
        addRequirements(swerve);
        addRequirements(arm);
        pidTurn = new PIDController(0.5, 0, 0);
        pidX = new PIDController(0.5, 0, 0);
        pidY = new PIDController(0.5, 0, 0);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        scorePosition = scorePositionSupplier.get();
        claw.resetTwistEncoder();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if(!wallBanged) {
            int divider = 2;
            if(NetworkTables.apriltagPositionLeft()[1] != 0 || NetworkTables.apriltagPositionLeft()[1] != 0)
                divider = 1;

            double average = (Math.abs(NetworkTables.apriltagPositionLeft()[1]) + Math.abs(NetworkTables.apriltagPositionRight()[1])) / divider;
            
            ySpeed = pidY.calculate(average, 0);
            if(pidY.atSetpoint())
                wallBanged = true;
            
        } else if(!centered) {
            int divider = 2;
            if(NetworkTables.apriltagPositionLeft()[0] != 0 || NetworkTables.apriltagPositionLeft()[0] != 0)
                divider = 1;

            double average = (Math.abs(NetworkTables.apriltagPositionLeft()[0]) + Math.abs(NetworkTables.apriltagPositionRight()[0])) / divider;


            
            xSpeed = pidX.calculate(average, CuboneManager.isConeInClaw() ? Constants.Vision.apriltagOffset : 0);

            if(pidX.atSetpoint())
                centered = true;

            

        } else {
            if(CuboneManager.isConeInClaw() && (scorePosition != 1 || scorePosition != 4 || scorePosition != 7)) {

                


            } else if(CuboneManager.isCubeInClaw() && (scorePosition == 1 || scorePosition == 4 || scorePosition == 7)) {
                if(!reachedArmAngle) {
                    switch(scorePosition) {
                        case 1: 
                        if(arm.setArmAngle(70) < 1)
                            reachedArmAngle = true;
                        break;
                        case 4:
                        if(arm.setArmAngle(30) < 1)
                            reachedArmAngle = true;
                        break;
                        case 7:
                        if(arm.setArmAngle(0) < 1)
                            reachedArmAngle = true;
                        break;
                    }
                } else if(!extendedArm) {
                    switch(scorePosition) {
                        case 1: 
                        if(arm.setElevatorPosition(100) < 1)
                            extendedArm = true;
                        break;
                        case 4:
                        if(arm.setElevatorPosition(50) < 1)
                            extendedArm = true;
                        break;
                        case 7:
                        if(arm.setElevatorPosition(10) < 1)
                            extendedArm = true;
                        break;
                    }
                } else {
                    claw.unGrab();
                }


            }
        }

        

        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);


        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        swerve.setModuleStates(moduleStates);


//System.out.println("Current angle: " + currentAngle + "\tAngle: " + power + "\tPower: " + power);


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        haveCone = false;
        haveCube = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
//time.getUsClock()() > 1 && Math.abs(NetworkTable.getAngle()) < 5?
//return isAngle;
        return false;
    }
}
