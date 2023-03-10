// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.sound.sampled.LineEvent;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.Ports;
import frc.robot.commands.GetCubone;
import frc.robot.commands.LineUpConeLeft;
import frc.robot.commands.LineUpConeRight;
import frc.robot.commands.LineUpCube;
import frc.robot.commands.SwerveCommand;
import frc.robot.commands.TurnToPosition;
import frc.robot.commands.ArmClaw.High.Cone.HighAnglesCone;
import frc.robot.commands.ArmClaw.High.Cone.HighExtendCone;
import frc.robot.commands.ArmClaw.High.Cone.HighFinishCone;
import frc.robot.commands.ArmClaw.High.Cube.HighAnglesCube;
import frc.robot.commands.ArmClaw.High.Cube.HighExtendCube;
import frc.robot.commands.ArmClaw.Low.Cone.LowAnglesCone;
import frc.robot.commands.ArmClaw.Low.Cone.LowExtendCone;
import frc.robot.commands.ArmClaw.Low.Cone.LowFinishCone;
import frc.robot.commands.ArmClaw.Low.Cube.LowAnglesCube;
import frc.robot.commands.ArmClaw.Low.Cube.LowExtendCube;
import frc.robot.commands.ArmClaw.Medium.Cone.MediumAnglesCone;
import frc.robot.commands.ArmClaw.Medium.Cone.MediumExtendCone;
import frc.robot.commands.ArmClaw.Medium.Cone.MediumFinishCone;
import frc.robot.commands.ArmClaw.Medium.Cube.MediumAnglesCube;
import frc.robot.commands.ArmClaw.Medium.Cube.MediumExtendCube;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CuboneManager;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  	// The robot's subsystems and commands are defined here...
  
	private final Claw claw = new Claw();
	private final Arm arm = new Arm();
	//private final CuboneManager cuboneManager = new CuboneManager();
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();


	private final TurnToPosition turnToPosition = new TurnToPosition(swerveSubsystem, 90);  	
	

	private final LineUpConeLeft lineUpConeLeft = new LineUpConeLeft(swerveSubsystem);
	private final LineUpConeRight lineUpConeRight = new LineUpConeRight(swerveSubsystem);
	private final LineUpCube lineUpCube = new LineUpCube(swerveSubsystem);

	private final HighAnglesCone highAnglesCone = new HighAnglesCone(arm, claw);
	private final HighExtendCone highExtendCone = new HighExtendCone(arm);
	private final HighFinishCone highFinishCone = new HighFinishCone(arm, claw);

	private final MediumAnglesCone mediumAnglesCone = new MediumAnglesCone(arm, claw);
	private final MediumExtendCone mediumExtendCone = new MediumExtendCone(arm);
	private final MediumFinishCone mediumFinishCone = new MediumFinishCone(arm, claw);

	private final LowAnglesCone lowAnglesCone = new LowAnglesCone(arm, claw);
	private final LowExtendCone lowExtendCone = new LowExtendCone(arm);
	private final LowFinishCone lowFinishCone = new LowFinishCone(arm, claw);

	private final HighAnglesCube highAnglesCube = new HighAnglesCube(arm, claw);
	private final HighExtendCube highExtendCube = new HighExtendCube(arm, claw);
	private final MediumAnglesCube mediumAnglesCube = new MediumAnglesCube(arm, claw);
	private final MediumExtendCube mediumExtendCube = new MediumExtendCube(arm, claw);
	private final LowAnglesCube lowAnglesCube = new LowAnglesCube(arm, claw);
	private final LowExtendCube lowExtendCube = new LowExtendCube(arm, claw);

	private final SequentialCommandGroup lineupHighConeLeft = new SequentialCommandGroup(highAnglesCone, lineUpConeLeft, highExtendCone, new WaitCommand(.5), highFinishCone);
	private final SequentialCommandGroup lineupHighCube = new SequentialCommandGroup(highAnglesCube, lineUpCube, highExtendCube);
	private final SequentialCommandGroup lineupHighConeRight = new SequentialCommandGroup(highAnglesCone, lineUpConeRight, highExtendCone,new WaitCommand(.5), highFinishCone);

	private final SequentialCommandGroup lineupMediumConeLeft = new SequentialCommandGroup(mediumAnglesCone, lineUpConeLeft, mediumExtendCone, new WaitCommand(.5), mediumFinishCone);
	private final SequentialCommandGroup lineupMediumCube = new SequentialCommandGroup(mediumAnglesCube, lineUpCube, mediumExtendCube);
	private final SequentialCommandGroup lineupMediumConeRight = new SequentialCommandGroup(mediumAnglesCone, lineUpConeRight, mediumExtendCone, new WaitCommand(.5), mediumFinishCone);

	private final SequentialCommandGroup lineupLowConeLeft = new SequentialCommandGroup(lowAnglesCone, lineUpConeLeft, lowExtendCone, new WaitCommand(.5), lowFinishCone);
	private final SequentialCommandGroup lineupLowCube = new SequentialCommandGroup(lowAnglesCube, lineUpCube, lowExtendCube);
	private final SequentialCommandGroup lineupLowConeRight = new SequentialCommandGroup(lowAnglesCone, lineUpConeRight, lowExtendCone, new WaitCommand(.5), lowFinishCone);

	


	private final GetCubone getCubone = new GetCubone(claw, swerveSubsystem, arm);
	//private final GetCubone rotateToCone = new GetCubone(claw, swerve, arm);
	private final CommandXboxController controller = new CommandXboxController(Ports.controller);
	private final CommandJoystick joystick = new CommandJoystick(4);
	private final XboxController joystickButtons = new XboxController(4);
	private final CommandXboxController buttonBoard = new CommandXboxController(1);
	private final Trigger button7 = new JoystickButton(joystickButtons, 7);
	private final Trigger button14 = new JoystickButton(joystickButtons, 14);

	//private final SequentialCommandGroup scoreCommand = new SequentialCommandGroup(lineUp, lineUp);
	//private final ParallelCommandGroup scoreParallel = new ParallelCommandGroup(lineUp, lineUp);

	private final CommandJoystick leftStick = new CommandJoystick(2);
	private final CommandJoystick rightStick = new CommandJoystick(3);

	private Command autoBot;

 
  
 // private final CommandXboxController buttonBoard = new CommandXboxController(Ports.buttonBoard);
  //private final Score score = new Score(claw, swerve, arm, () -> 8);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

	public RobotContainer() {

		PathPlannerTrajectory path = PathPlanner.loadPath("epic", new PathConstraints(3, 3));


		// This is just an example event map. It would be better to have a constant, global event map
		// in your code that will be used by all path following commands.
		HashMap<String, Command> eventMap = new HashMap<String, Command>();
	
	
		eventMap.put("marker1", new PrintCommand("Passed marker 1"));
		eventMap.put("pickUp", new WaitCommand(5));
		//eventMap.put("lineUp", lineUp);
		eventMap.put("score", new PrintCommand("Scoring"));
	
		
		// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
		SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
			swerveSubsystem::getPose, // Pose2d supplier
			swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
			swerveSubsystem.kinematics, // SwerveDriveKinematics
			new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
			new PIDConstants(2.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
			swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
			eventMap,
			true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
			swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
		);
		
		autoBot = autoBuilder.fullAuto(path);

		/* 
		swerveSubsystem.setDefaultCommand(new SwerveCommand(
			swerveSubsystem,
			() -> controller.getLeftY(),
			//() -> driverJoystick.getY(),
			//() -> driverJoystick.getX(),
			() -> controller.getLeftX(),
			() -> controller.getRightX()));
			//() -> driverJoystick2.getX(),
		*/
		
		/*
		swerveSubsystem.setDefaultCommand(new SwerveCommand(swerveSubsystem, 
		() -> joystick.getY(), 
		() -> joystick.getX(), 
		() -> joystick.getTwist(), 
		() -> joystick.getRawAxis(3),
		() -> joystickButtons.getRawButton(2),
		() -> joystickButtons.getRawButton(1),
		5
		));
		*/
		
		swerveSubsystem.setDefaultCommand(new SwerveCommand(swerveSubsystem,
		() -> leftStick.getY(), 
		() -> leftStick.getX(), 
		() -> rightStick.getTwist(),
		() -> rightStick.getX(),
		() -> rightStick.getY(),
		() -> rightStick.button(1).getAsBoolean(),
		() -> leftStick.button(1).getAsBoolean(),
		() -> leftStick.button(2).getAsBoolean()
		));
		
		configureButtonBindings();
	}

	/**
	* Use this method to define your button->command mappings. Buttons can be created by
	* instantiating a {@link GenericHID} or one of its subclasses ({@link
	* edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
	* edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	*/
  	private void configureButtonBindings() {
		//new JoystickButton(controller, 0).onTrue(rotateToCone);

		// Schedule `ExampleCommand` when `exampleCondition` changes to `true`
		//new Trigger(m_drivetrain::exampleCondition).onTrue(new DriveCommand(m_drivetrain));

		// Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
		// cancelling on release.
		//controller.b().whileTrue(rotateToCone);
		
		//controller.x().whileTrue(score);
		controller.y().whileTrue(Commands.run(() -> arm.winchUpPos()));
		//controller.y().whileTrue())
		controller.a().whileTrue(Commands.run(() -> arm.winchDownPos()));
		controller.b().whileTrue(Commands.run(() -> arm.extendPos()));
		controller.x().whileTrue(Commands.run(() -> arm.retractPos()));
		controller.leftBumper().whileTrue(Commands.runOnce(() -> claw.toggleGrab()));
		controller.rightBumper().whileTrue(getCubone);
		controller.start().whileTrue(lineupHighCube);
		controller.povUp().whileTrue(Commands.run(() -> claw.wristUpPos()));
		controller.povDown().whileTrue(Commands.run(() -> claw.wristDownPos()));
		//controller.povRight().whileTrue(Commands.startEnd(() -> claw.twistClaw(0.5), () -> claw.twistClaw(0)).repeatedly());
		controller.povRight().whileTrue(Commands.run(() -> claw.twistUpPos()));
		controller.povLeft().whileTrue(Commands.run(() -> claw.twistDownPos()));
		//controller.povLeft().whileTrue(Commands.startEnd(() -> claw.setTwistAngle(45), () -> claw.twistClaw(0)).repeatedly());
		//leftStick.button(2).onTrue(Commands.runOnce(() -> NavX.reset()));
		///rightStick.button(1).onTrue(Commands.runOnce(() -> swerveSubsystem.resetEncoders()));
		rightStick.button(1).onTrue(Commands.runOnce(() -> NavX.reset()));
		rightStick.button(2).onTrue(Commands.runOnce(() -> swerveSubsystem.resetEncoders(), swerveSubsystem));

		//button7.onTrue(Commands.runOnce(() -> NavX.reset()));
		//button14.onTrue(Commands.runOnce(() -> swerveSubsystem.resetEncoders(), swerveSubsystem));
		//joystick.povUp().onTrue(turnToPosition);
		//controller.b().whileTrue(Commands.run(() -> arm.winchUp(), arm));
		//controller.x().whileTrue(RepeatCommand()))

		//Buttons

		buttonBoard.button(0).whileTrue(lineupLowConeLeft);
		buttonBoard.button(1).whileTrue(lineupLowCube);
		buttonBoard.button(2).whileTrue(lineupLowConeRight);

		buttonBoard.button(3).whileTrue(lineupMediumConeLeft);
		buttonBoard.button(4).whileTrue(lineupMediumCube);
		buttonBoard.button(5).whileTrue(lineupMediumConeRight);

		buttonBoard.button(6).whileTrue(lineupHighConeLeft);
		buttonBoard.button(7).whileTrue(lineupHighCube);
		buttonBoard.button(8).whileTrue(lineupHighConeRight);


	}

	/**
	* Use this to pass the autonomous command to the main {@link Robot} class.
	*
	* @return the command to run in autonomous
	*/
	public Command getAutonomousCommand() {
    
		PathPlannerTrajectory examplePath = PathPlanner.loadPath("Test Path", new PathConstraints(3, 3));
		Constraints angleConstraints = new Constraints(1.2, 1.2);
		PIDController xPID = new PIDController(0.5, 0, 0);
		PIDController yPID = new PIDController(0.5, 0, 0);
		ProfiledPIDController anglePID = new ProfiledPIDController(5, 0, 0, angleConstraints);
		anglePID.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
			examplePath, 
			swerveSubsystem::getPose,
			Constants.DriveConstants.DRIVE_KINEMATICS, 
			xPID, yPID, anglePID, 
			swerveSubsystem::setModuleStates, 
			swerveSubsystem);

		
		/* 
    	return new SequentialCommandGroup(
			//new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
			new InstantCommand(() -> swerveSubsystem.resetOdometry(examplePath.getInitialHolonomicPose())),
			swerveControllerCommand,
			new InstantCommand(() -> swerveSubsystem.stopModules())
		); */

		return autoBot;
	
	}
}