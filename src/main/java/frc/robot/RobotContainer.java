// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.Ports;
import frc.robot.commands.ApriltagFollow;
import frc.robot.commands.GetCubone;
import frc.robot.commands.LineUp;
import frc.robot.commands.Score;
import frc.robot.commands.SwerveCommand;
import frc.robot.commands.TurnToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CuboneManager;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
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
  //private final GetCubone rotateToCone = new GetCubone(claw, swerve, arm);
  private final CommandXboxController controller = new CommandXboxController(Ports.controller);
  private final CommandJoystick joystick = new CommandJoystick(1);
  private final XboxController joystickButtons = new XboxController(1);
  private final Trigger button7 = new JoystickButton(joystickButtons, 7);
  private final Trigger button14 = new JoystickButton(joystickButtons, 14);

  private final CommandJoystick leftStick = new CommandJoystick(2);
  private final CommandJoystick rightStick = new CommandJoystick(3);

  private Command autoBot;

 
  
 // private final CommandXboxController buttonBoard = new CommandXboxController(Ports.buttonBoard);
  //private final Score score = new Score(claw, swerve, arm, () -> 8);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

	public RobotContainer() {

		PathPlannerTrajectory path = PathPlanner.loadPath("epic", new PathConstraints(4, 3));


		// This is just an example event map. It would be better to have a constant, global event map
		// in your code that will be used by all path following commands.
		HashMap<String, Command> eventMap = new HashMap<String, Command>();
	
	
		eventMap.put("marker1", new PrintCommand("Passed marker 1"));
	
		
		// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
		SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
			swerveSubsystem::getPose, // Pose2d supplier
			swerveSubsystem::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
			swerveSubsystem.kinematics, // SwerveDriveKinematics
			new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
			new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
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
		() -> leftStick.getTwist(),
		() -> rightStick.getX(),
		() -> rightStick.getY(),
		() -> leftStick.button(1).getAsBoolean()
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
		controller.y().whileTrue(Commands.run(() -> arm.winchUpPos(), arm));
		//controller.y().whileTrue())
		controller.a().whileTrue(Commands.run(() -> arm.winchDownPos(), arm));
		controller.b().whileTrue(Commands.run(() -> arm.extendPos(), arm));
		controller.x().whileTrue(Commands.run(() -> arm.retractPos(), arm));
		controller.leftBumper().whileTrue(Commands.runOnce(() -> claw.toggleGrab(), claw));
    controller.start().whileTrue(new LineUp(swerveSubsystem));
		controller.povUp().whileTrue(Commands.run(() -> claw.wristUpPos(), claw));
		controller.povDown().whileTrue(Commands.run(() -> claw.wristDownPos(), claw));
		controller.povRight().whileTrue(Commands.startEnd(() -> claw.twistClaw(.2), () -> claw.twistClaw(0),  claw).repeatedly());
		controller.povLeft().whileTrue(Commands.startEnd(() -> claw.twistClaw(-.2), () -> claw.twistClaw(0),  claw).repeatedly());
		//leftStick.button(1/*TODO: find the button that i can use*/).whileTrue(new LineUp(swerveSubsystem));
		//leftStick.button(2).onTrue(Commands.runOnce(() -> NavX.reset()));
		///rightStick.button(1).onTrue(Commands.runOnce(() -> swerveSubsystem.resetEncoders()));
		leftStick.button(1).onTrue(Commands.runOnce(() -> NavX.reset()));
		leftStick.button(2).onTrue(Commands.runOnce(() -> swerveSubsystem.resetEncoders(), swerveSubsystem));

		//button7.onTrue(Commands.runOnce(() -> NavX.reset()));
		//button14.onTrue(Commands.runOnce(() -> swerveSubsystem.resetEncoders(), swerveSubsystem));
		//joystick.povUp().onTrue(turnToPosition);
		//controller.b().whileTrue(Commands.run(() -> arm.winchUp(), arm));
		//controller.x().whileTrue(RepeatCommand()))

	}

	/**
	* Use this to pass the autonomous command to the main {@link Robot} class.
	*
	* @return the command to run in autonomous
	*/
	public Command getAutonomousCommand() {
		TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2,
		 1).setKinematics(Constants.DriveConstants.DRIVE_KINEMATICS);


		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
			new Pose2d(0, 0, new Rotation2d(0)),
			List.of(
				new Translation2d(1, 0),
				new Translation2d(1, -1)
			),
			new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
			trajectoryConfig
		);
    
		PIDController xPID = new PIDController(0.5, 0, 0);
		PIDController yPID = new PIDController(0.5, 0, 0);
		ProfiledPIDController anglePID = new ProfiledPIDController(0.5, 0, 0, null);
		anglePID.enableContinuousInput(-Math.PI, Math.PI);

		
    return null;
	}
}
