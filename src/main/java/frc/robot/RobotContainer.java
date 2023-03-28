// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.Ports;
import frc.robot.commands.Balance;
import frc.robot.commands.SwerveCommand;
import frc.robot.commands.TurnToPosition;
import frc.robot.commands.ArmClaw.CubePickup;
import frc.robot.commands.ArmClaw.ResetArmClaw;
import frc.robot.commands.ArmClaw.SetAngle;
import frc.robot.commands.ArmClaw.High.HighAngles;
import frc.robot.commands.ArmClaw.High.HighExtend;
import frc.robot.commands.ArmClaw.High.HighDrop;
import frc.robot.commands.ArmClaw.High.HighRetract;
import frc.robot.commands.ButtonCommands.AttackAngle;
import frc.robot.commands.ButtonCommands.DrivingAngle;
import frc.robot.commands.ButtonCommands.ExtendFar;
import frc.robot.commands.ButtonCommands.ExtendShort;
import frc.robot.commands.ButtonCommands.IntakeParshell;
import frc.robot.commands.ButtonCommands.IntakePerpendiskular;
import frc.robot.commands.ButtonCommands.LiftAngleHigh;
import frc.robot.commands.ButtonCommands.LiftAngleMid;
import frc.robot.commands.ButtonCommands.PickUpAngle;
import frc.robot.commands.ButtonCommands.StopAllArmAndClaw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

	private Command scoreAndParkLong;
	private Command scoreAndParkClose;
	private Command score2Feeder;
	private Command justScore;
	private Command scoreAndBalance;


 
	private ShuffleboardTab tab = Shuffleboard.getTab("AutoChooser");
    private GenericEntry autoChoice = tab.add("Auto Choice", 0.0).getEntry();
    
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

	public RobotContainer() {

		PathPlannerTrajectory scoreAndParkClosePath = PathPlanner.loadPath("ScoreAndParkClose", new PathConstraints(1, 2));
		PathPlannerTrajectory scoreAndParkLongPath = PathPlanner.loadPath("ScoreAndParkLong", new PathConstraints(1, 2));
		PathPlannerTrajectory score2FeederPath = PathPlanner.loadPath("Score2Feeder", new PathConstraints(3.2, 2));
		//PathPlannerTrajectory score2FeederPath = PathPlanner.loadPath("Score2Feeder", new PathConstraints(1, 2));


		// This is just an example event map. It would be better to have a constant, global event map
		// in your code that will be used by all path following commands.
		HashMap<String, Command> scoreAndParkCloseMap = new HashMap<String, Command>();
		HashMap<String, Command> scoreAndParkLongMap = new HashMap<String, Command>();
		HashMap<String, Command> score2FeederMap = new HashMap<String, Command>();

	
		scoreAndParkCloseMap.put("score", new SequentialCommandGroup(Commands.runOnce(() -> claw.grab()), new HighAngles(arm, claw), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new WaitCommand(0.5), new HighAngles(arm, claw), new HighRetract(arm, claw)));
		scoreAndParkLongMap.put("score", new SequentialCommandGroup(Commands.runOnce(() -> claw.grab()), new HighAngles(arm, claw), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new WaitCommand(0.5), new HighAngles(arm, claw), new HighRetract(arm, claw)));

		score2FeederMap.put("score", new SequentialCommandGroup(Commands.runOnce(() -> claw.grab()), new HighAngles(arm, claw), new HighExtend(arm),new WaitCommand(.12), new HighDrop(arm, claw), new WaitCommand(0.12), new HighAngles(arm, claw), new HighRetract(arm, claw)));
		score2FeederMap.put("readyToGrab", new CubePickup(claw, arm));
		//score2FeederMap.put("pickUp", new SequentialCommandGroup(new SetAngle(claw, arm), Commands.runOnce(() -> claw.unGrab()), new LineUpSwerveCone(claw, swerveSubsystem), new LineUpClaw(claw), new WaitCommand(0.25), new Attack(claw, arm), new WaitCommand(.5), new LineUpRetract(arm, claw)));
		score2FeederMap.put("angles", new HighAngles(arm, claw));
		score2FeederMap.put("score2", new SequentialCommandGroup(Commands.runOnce(() -> swerveSubsystem.stopModules()), new HighAngles(arm, claw), new HighExtend(arm),new WaitCommand(.08), new HighDrop(arm, claw), new WaitCommand(0.12), new HighAngles(arm, claw), new HighRetract(arm, claw)));
		

		// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.


		SwerveAutoBuilder scoreAndParkCloseBuilder = new SwerveAutoBuilder(
			swerveSubsystem::getPose, // Pose2d supplier
			swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
			swerveSubsystem.kinematics, // SwerveDriveKinematics
			new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
			new PIDConstants(2.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
			swerveSubsystem::setAutoModuleStates, // Module states consumer used to output to the drive subsystem
			scoreAndParkCloseMap,
			true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
			swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
		);
		SwerveAutoBuilder scoreAndParkLongBuilder = new SwerveAutoBuilder(
			swerveSubsystem::getPose, // Pose2d supplier
			swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
			swerveSubsystem.kinematics, // SwerveDriveKinematics
			new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
			new PIDConstants(2.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
			swerveSubsystem::setAutoModuleStates, // Module states consumer used to output to the drive subsystem
			scoreAndParkLongMap,
			true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
			swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
		);
		SwerveAutoBuilder score2FeederBuilder = new SwerveAutoBuilder(
			swerveSubsystem::getPose, // Pose2d supplier
			swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
			swerveSubsystem.kinematics, // SwerveDriveKinematics
			new PIDConstants(15, 0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
			new PIDConstants(2.2, 0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
			swerveSubsystem::setAutoModuleStates, // Module states consumer used to output to the drive subsystem
			score2FeederMap,
			true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
			swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
		);

		
		scoreAndParkClose = scoreAndParkCloseBuilder.fullAuto(scoreAndParkClosePath);
		scoreAndParkLong = scoreAndParkLongBuilder.fullAuto(scoreAndParkLongPath);
		score2Feeder = score2FeederBuilder.fullAuto(score2FeederPath);
		justScore = new SequentialCommandGroup(Commands.runOnce(() -> claw.grab()), new HighAngles(arm, claw), new HighExtend(arm), new WaitCommand(.5), new HighDrop(arm, claw), new WaitCommand(0.5), new HighAngles(arm, claw), new HighRetract(arm, claw));


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

		controller.y().whileTrue(Commands.run(() -> arm.winchUpPos()));
		controller.a().whileTrue(Commands.run(() -> arm.winchDownPos()));
		controller.b().whileTrue(Commands.run(() -> arm.extendPos()));
		controller.x().whileTrue(Commands.run(() -> arm.retractPos()));
		controller.leftBumper().onTrue(new SequentialCommandGroup(Commands.runOnce(() -> claw.unGrab()), new WaitCommand(2), Commands.runOnce(() -> claw.stopGrab())));
		controller.rightBumper().onTrue(Commands.runOnce(() -> claw.toggleGrab()));
		controller.start().whileTrue(new SetAngle(claw, arm));
		controller.povUp().whileTrue(Commands.run(() -> claw.wristUpPos()));
		controller.povDown().whileTrue(Commands.run(() -> claw.wristDownPos()));
		controller.povRight().whileTrue(Commands.run(() -> claw.twistUpPos()));
		controller.povLeft().whileTrue(Commands.run(() -> claw.twistDownPos()));
		controller.leftStick().onTrue(Commands.run(() -> claw.killTwist()));
		controller.rightStick().onTrue(Commands.run(() -> claw.unKillTwist()));
		rightStick.button(1).onTrue(Commands.runOnce(() -> NavX.reset()));
		rightStick.button(2).onTrue(Commands.runOnce(() -> swerveSubsystem.resetEncoders(), swerveSubsystem));

		//Buttons

		buttonBoard.button(1).whileTrue(new LiftAngleHigh(claw, arm));
		buttonBoard.button(2).whileTrue(new LiftAngleMid(claw, arm));
		buttonBoard.button(3).whileTrue(new ExtendFar(claw, arm));
		buttonBoard.button(4).whileTrue(new ExtendShort(claw, arm));

		//buttonBoard.button(5).whileTrue(lineupMediumConeRight); LED
		buttonBoard.button(6).whileTrue(new CubePickup(claw, arm));

		buttonBoard.button(7).onTrue(Commands.runOnce(() -> claw.grab()));
		buttonBoard.button(8).onTrue(Commands.runOnce(() -> claw.unGrab()));
		buttonBoard.button(9).whileTrue(new IntakePerpendiskular(claw, arm));
		buttonBoard.button(10).whileTrue(new PickUpAngle(claw, arm));
		buttonBoard.button(11).whileTrue(new IntakeParshell(claw, arm));
		buttonBoard.button(12).whileTrue(new StopAllArmAndClaw(claw, arm));
		buttonBoard.button(13).onTrue(new AttackAngle(claw, arm));
		buttonBoard.button(14).whileTrue(new DrivingAngle(claw, arm));
	}

	/**
	* Use this to pass the autonomous command to the main {@link Robot} class.
	*
	* @return the command to run in autonomous
	*/
	public Command getAutonomousCommand() {

		if(autoChoice.getDouble(0.0) == 1.0) {
			return scoreAndParkClose;
		}
		else if(autoChoice.getDouble(0.0) == 2.0) {
			return scoreAndParkLong;
		}
		else if(autoChoice.getDouble(0.0) == 3.0) {
			return score2Feeder;
		}
		else if(autoChoice.getDouble(0.0) == 4.0) {
			return null; //balance
		}
		else if(autoChoice.getDouble(0.0) == 2169) {
			return null;
		}
		return justScore;
	
	}
}