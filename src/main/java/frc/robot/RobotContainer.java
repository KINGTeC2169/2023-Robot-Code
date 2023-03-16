// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import javax.print.attribute.standard.MediaSize.NA;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.Ports;
import frc.robot.commands.Balance;
import frc.robot.commands.LineUpConeLeft;
import frc.robot.commands.LineUpConeRight;
import frc.robot.commands.LineUpCube;
import frc.robot.commands.SwerveCommand;
import frc.robot.commands.TurnToPosition;
import frc.robot.commands.ArmClaw.ResetArmClaw;
import frc.robot.commands.ArmClaw.High.HighAngles;
import frc.robot.commands.ArmClaw.High.HighExtend;
import frc.robot.commands.ArmClaw.High.HighDrop;
import frc.robot.commands.ArmClaw.High.HighRetract;
import frc.robot.commands.ArmClaw.Low.LowAnglesCone;
import frc.robot.commands.ArmClaw.Low.LowExtendCone;
import frc.robot.commands.ArmClaw.Low.LowFinishCone;
import frc.robot.commands.ArmClaw.Medium.MediumAngles;
import frc.robot.commands.ArmClaw.Medium.MediumDrop;
import frc.robot.commands.GetStuff.Attack;
import frc.robot.commands.GetStuff.LineUpClaw;
import frc.robot.commands.GetStuff.LineUpRetract;
import frc.robot.commands.GetStuff.LineUpSwerveCone;
import frc.robot.commands.GetStuff.LineUpSwerveCube;
import frc.robot.commands.GetStuff.SetAngle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
	private final NavX navx = new NavX();
	//private final CuboneManager cuboneManager = new CuboneManager();
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();


	private final TurnToPosition turnToPosition = new TurnToPosition(swerveSubsystem, 90);  	
	
	private final ResetArmClaw resetArmClaw = new ResetArmClaw(arm);


	private final LineUpSwerveCone lineUpSwerveCone = new LineUpSwerveCone(claw, swerveSubsystem);
	
	private final LineUpSwerveCube lineUpSwerveCube = new LineUpSwerveCube(claw, swerveSubsystem);
	private final SetAngle setAngle = new SetAngle(claw, arm);
	private final Attack attack = new Attack(claw, arm);
	private final LineUpClaw lineUpClaw = new LineUpClaw(claw);
	private final Balance balance = new Balance(swerveSubsystem);

	private final SequentialCommandGroup lineupHighConeLeft = new SequentialCommandGroup(new HighAngles(arm, claw), new LineUpConeLeft(swerveSubsystem), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new HighRetract(arm, claw));
	private final SequentialCommandGroup lineupHighCube = new SequentialCommandGroup(new HighAngles(arm, claw), new LineUpCube(swerveSubsystem), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new HighRetract(arm, claw));
	private final SequentialCommandGroup lineupHighConeRight = new SequentialCommandGroup(new HighAngles(arm, claw), new LineUpConeRight(swerveSubsystem), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new HighRetract(arm, claw));

	private final SequentialCommandGroup lineupMediumConeLeft = new SequentialCommandGroup(new MediumAngles(arm, claw), new LineUpConeLeft(swerveSubsystem), new MediumDrop(claw));
	private final SequentialCommandGroup lineupMediumCube = new SequentialCommandGroup(new MediumAngles(arm, claw), new LineUpCube(swerveSubsystem), new MediumDrop(claw));
	private final SequentialCommandGroup lineupMediumConeRight = new SequentialCommandGroup(new MediumAngles(arm, claw), new LineUpConeRight(swerveSubsystem), new MediumDrop(claw));

	//private final SequentialCommandGroup lineupLowConeLeft = new SequentialCommandGroup(new LowAnglesCone(arm, claw), new LineUpConeLeft(swerveSubsystem), new LowExtendCone(arm),new WaitCommand(.5), new LowFinishCone(arm, claw));
	//private final SequentialCommandGroup lineupLowCube = new SequentialCommandGroup(new LowAnglesCube(arm, claw), new LineUpCube(swerveSubsystem), new LowExtendCube(arm, claw));
	//private final SequentialCommandGroup lineupLowConeRight = new SequentialCommandGroup(new LowAnglesCone(arm, claw), new LineUpConeRight(swerveSubsystem), new LowExtendCone(arm),new WaitCommand(.5), new LowFinishCone(arm, claw));
	

	private final SequentialCommandGroup getConeCommand = new SequentialCommandGroup(new SetAngle(claw, arm), lineUpSwerveCone, new LineUpClaw(claw), new WaitCommand(0.5), new Attack(claw, arm), new WaitCommand(.5), new LineUpRetract(arm, claw));
	private final SequentialCommandGroup getCubeCommand = new SequentialCommandGroup(new SetAngle(claw, arm), lineUpSwerveCube, new LineUpClaw(claw), new WaitCommand(0.5), attack, new WaitCommand(.5), new LineUpRetract(arm, claw));
	


	
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

	private Command score2;
	private Command score2NoPark;

 
  
 // private final CommandXboxController buttonBoard = new CommandXboxController(Ports.buttonBoard);
  //private final Score score = new Score(claw, swerve, arm, () -> 8);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

	public RobotContainer() {

		PathPlannerTrajectory score2Path = PathPlanner.loadPath("Score2", new PathConstraints(2, 1));
		PathPlannerTrajectory score2NoParkPath = PathPlanner.loadPath("score2NoPark", new PathConstraints(1, 3));

		// This is just an example event map. It would be better to have a constant, global event map
		// in your code that will be used by all path following commands.
		HashMap<String, Command> score2Map = new HashMap<String, Command>();
		HashMap<String, Command> score2NoParkMap = new HashMap<String, Command>();
	
		score2Map.put("score", new SequentialCommandGroup(new HighAngles(arm, claw), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new HighRetract(arm, claw)));
		score2Map.put("pickUp", new SequentialCommandGroup(new SetAngle(claw, arm), lineUpSwerveCone, new LineUpClaw(claw), new WaitCommand(0.5), new Attack(claw, arm), new WaitCommand(.5), new LineUpRetract(arm, claw)));
		score2Map.put("score2", new SequentialCommandGroup(new HighAngles(arm, claw), new LineUpConeRight(swerveSubsystem), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new HighRetract(arm, claw)));
		//eventMap.put("lineUp", lineUp);
		score2NoParkMap.put("scoreCone", new SequentialCommandGroup(new HighAngles(arm, claw), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new HighRetract(arm, claw)));
		score2NoParkMap.put("trainingWheels", new PrintCommand("I print things \n balls \n balls"));
		score2NoParkMap.put("balance", new PrintCommand("I print things as well \n cock \n cock"));

		
		// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
		SwerveAutoBuilder score2Builder = new SwerveAutoBuilder(
			swerveSubsystem::getPose, // Pose2d supplier
			swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
			swerveSubsystem.kinematics, // SwerveDriveKinematics
			new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
			new PIDConstants(2.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
			swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
			score2Map,
			true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
			swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
		);
		SwerveAutoBuilder autoBuilder2 = new SwerveAutoBuilder(
			swerveSubsystem::getPose, // Pose2d supplier
			swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
			swerveSubsystem.kinematics, // SwerveDriveKinematics
			new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
			new PIDConstants(2.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
			swerveSubsystem::setAutoModuleStates, // Module states consumer used to output to the drive subsystem
			score2NoParkMap,
			true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
			swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
		);
		
		score2 = score2Builder.fullAuto(score2Path);
		score2NoPark = autoBuilder2.fullAuto(score2NoParkPath);


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

		buttonBoard.button(3).whileTrue(getConeCommand);
		//buttonBoard.button(1).whileTrue(lineupLowCube);
		buttonBoard.button(1).whileTrue(getCubeCommand);

		buttonBoard.button(4).whileTrue(lineupMediumConeLeft);
		buttonBoard.button(5).whileTrue(lineupMediumCube);
		buttonBoard.button(6).whileTrue(lineupMediumConeRight);

		buttonBoard.button(7).whileTrue(lineupHighConeLeft);
		buttonBoard.button(8).whileTrue(lineupHighCube);
		buttonBoard.button(9).whileTrue(lineupHighConeRight);

		
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

		return score2NoPark;
	
	}
}