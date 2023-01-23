package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Motors;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.NetworkTables;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Score extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Claw claw;
    private final SwerveSubsystem swerve;
    private final Arm arm;


    private final Timer time;
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

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public Score(Claw claw, SwerveSubsystem swerve, Arm arm) {
        this.claw = claw;
        this.swerve = swerve;
        this.arm = arm;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(claw);
        addRequirements(swerve);
        addRequirements(arm);
        pidTurn = new PIDController(0.5, 0, 0);
        pidX = new PIDController(0.5, 0, 0);
        pidY = new PIDController(0.5, 0, 0);
        time = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        time.start();
        claw.resetTwistEncoder();
        angle = NetworkTables.getPalmAngle("Cone");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //angle = NetworkTables.getAngle();
        if (!haveCone && !haveCube) {
            if (NetworkTables.getPalmCenter("Cone")[0] != 0)
                haveCone = true;
            else if (NetworkTables.getPalmCenter("Cube")[0] != 0)
                haveCube = true;
        }


        //  x/2048 * 360
        if (haveCone) {
            xSpeed = pidX.calculate(NetworkTables.getPalmCenter("Cone")[0], 0);//setpoint should be whatever the center of the image is
            ySpeed = pidY.calculate(NetworkTables.getPalmCenter("Cone")[1], 0);
            claw.twistClaw(pidTurn.calculate(NetworkTables.getPalmAngle("Cone"), 0));
        } else if (haveCube) {
            xSpeed = pidX.calculate(NetworkTables.getPalmCenter("Cube")[0], 0);//setpoint should be whatever the center of the image is
            ySpeed = pidY.calculate(NetworkTables.getPalmCenter("Cube")[1], 0);
        } else {

            turningSpeed = pidTurn.calculate(NetworkTables.getFrontCenter()[0], 0);// center of image
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
