// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CuboneManager;
import frc.robot.subsystems.NetworkTables;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class GetCubone extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Claw claw;
	private final SwerveSubsystem swerve;
	private final Arm arm;


	private double xSpeed;
	private double ySpeed;
	private double turningSpeed;
	private boolean gottem;
	private boolean itemCentered;
	private ChassisSpeeds chassisSpeeds;
	
	private final PIDController pidTurn;
	private final PIDController pidX;
	private final PIDController pidY;
	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public GetCubone(Claw claw, SwerveSubsystem swerve, Arm arm) {
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
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		claw.resetTwistEncoder();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		//if(!CuboneManager.isSomethingInClaw()) {
			CuboneManager.setConeInClaw(NetworkTables.getPalmCenter("Cone")[0] != 0);
			CuboneManager.setCubeInClaw(NetworkTables.getPalmCenter("Cube")[0] != 0);
		//}
		if(!itemCentered) {
			if(CuboneManager.isConeInClaw()) {
				xSpeed = pidX.calculate(NetworkTables.getPalmCenter("Cone")[0], 240);
				ySpeed = pidY.calculate(NetworkTables.getPalmCenter("Cone")[1], 240);
				claw.twistClaw(pidTurn.calculate(NetworkTables.getPalmAngle("Cone"), 0));
				itemCentered = pidX.atSetpoint() && pidY.atSetpoint() && pidTurn.atSetpoint();
			}
			else if(CuboneManager.isCubeInClaw()) {
				xSpeed = pidX.calculate(NetworkTables.getPalmCenter("Cube")[0], 0);
				ySpeed = pidY.calculate(NetworkTables.getPalmCenter("Cube")[1], 0);
				itemCentered = pidX.atSetpoint() && pidY.atSetpoint();
			}
			else if(CuboneManager.isSomethingInFront() || !NetworkTables.isThereObjectPalm()) {
				turningSpeed = pidX.calculate(NetworkTables.closestObject()[0], 240);
				ySpeed = pidY.calculate(NetworkTables.closestObject()[1], 0);
			}
		}
		else {
			if(arm.setArmAngle(0) < 1000 && !gottem) {
				claw.grab();
				gottem = true;
			}
			if(gottem) {
				if(arm.setArmAngle(30) < 1000 && claw.setTwistAngle(0) < 1) {
					arm.winchStop();
					claw.twistClaw(0);
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
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
