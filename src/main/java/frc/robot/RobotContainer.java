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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.Ports;
import frc.robot.commands.Balance;
import frc.robot.commands.CoolBalance;
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
import frc.robot.commands.ButtonCommands.AttackAngle;
import frc.robot.commands.ButtonCommands.DrivingAngle;
import frc.robot.commands.ButtonCommands.ExtendFar;
import frc.robot.commands.ButtonCommands.ExtendShort;
import frc.robot.commands.ButtonCommands.FlipIntake;
import frc.robot.commands.ButtonCommands.IntakeParshell;
import frc.robot.commands.ButtonCommands.IntakePerpendiskular;
import frc.robot.commands.ButtonCommands.LiftAngleHigh;
import frc.robot.commands.ButtonCommands.LiftAngleMid;
import frc.robot.commands.ButtonCommands.PickUpAngle;
import frc.robot.commands.ButtonCommands.StopAllArmAndClaw;
import frc.robot.commands.GetStuff.Attack;
import frc.robot.commands.GetStuff.AttackUpright;
import frc.robot.commands.GetStuff.LineUpClaw;
import frc.robot.commands.GetStuff.LineUpRetract;
import frc.robot.commands.GetStuff.LineUpSwerveCone;
import frc.robot.commands.GetStuff.LineUpSwerveConeUpright;
import frc.robot.commands.GetStuff.LineUpSwerveCube;
import frc.robot.commands.GetStuff.SetAngle;
import frc.robot.commands.ScoreLineUp.LineUpConeRight;
import frc.robot.commands.ScoreLineUp.LineUpCube;
import frc.robot.commands.ScoreLineUp.SwerveLineUpConeLeft;
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

	private final SequentialCommandGroup lineupHighConeLeft = new SequentialCommandGroup(new HighAngles(arm, claw), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new HighRetract(arm, claw));
	private final SequentialCommandGroup lineupHighCube = new SequentialCommandGroup(new HighAngles(arm, claw), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new HighRetract(arm, claw));
	private final SequentialCommandGroup lineupHighConeRight = new SequentialCommandGroup(new HighAngles(arm, claw), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new HighRetract(arm, claw));

	private final SequentialCommandGroup lineupMediumConeLeft = new SequentialCommandGroup(new MediumAngles(arm, claw),/*, new LineUpConeLeft(swerveSubsystem)*/ new MediumDrop(claw));
	private final SequentialCommandGroup lineupMediumCube = new SequentialCommandGroup(new MediumAngles(arm, claw)/* , new LineUpCube(swerveSubsystem)*/, new MediumDrop(claw));
	private final SequentialCommandGroup lineupMediumConeRight = new SequentialCommandGroup(new MediumAngles(arm, claw), /*new LineUpConeRight(swerveSubsystem),*/ new MediumDrop(claw));

	//private final SequentialCommandGroup lineupLowConeLeft = new SequentialCommandGroup(new LowAnglesCone(arm, claw), new LineUpConeLeft(swerveSubsystem), new LowExtendCone(arm),new WaitCommand(.5), new LowFinishCone(arm, claw));
	//private final SequentialCommandGroup lineupLowCube = new SequentialCommandGroup(new LowAnglesCube(arm, claw), new LineUpCube(swerveSubsystem), new LowExtendCube(arm, claw));
	//private final SequentialCommandGroup lineupLowConeRight = new SequentialCommandGroup(new LowAnglesCone(arm, claw), new LineUpConeRight(swerveSubsystem), new LowExtendCone(arm),new WaitCommand(.5), new LowFinishCone(arm, claw));
	

	private final SequentialCommandGroup getConeCommand = new SequentialCommandGroup(new SetAngle(claw, arm), new LineUpSwerveCone(claw, swerveSubsystem), new LineUpClaw(claw), new WaitCommand(0.5), new Attack(claw, arm), new WaitCommand(.5), new LineUpRetract(arm, claw));
	private final SequentialCommandGroup getCubeCommand = new SequentialCommandGroup(new SetAngle(claw, arm), lineUpSwerveCube, new LineUpClaw(claw), new WaitCommand(0.5), new Attack(claw, arm), new WaitCommand(.5), new LineUpRetract(arm, claw));
	private final SequentialCommandGroup getConeUprightCommand = new SequentialCommandGroup(new SetAngle(claw, arm), new LineUpSwerveConeUpright(claw, swerveSubsystem), new WaitCommand(0.5), new AttackUpright(claw, arm), new WaitCommand(.5), new LineUpRetract(arm, claw));


	
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
	private Command scoreAndBalance;
	private Command scoreAndParkLong;
	private Command scoreAndParkClose;
	private Command score2Feeder;
	private Command justScore;
	private Command balanceMiddle;
	private Command basicBalance;

 
	private ShuffleboardTab tab = Shuffleboard.getTab("AutoChooser");
    private GenericEntry autoChoice = tab.add("Auto Choice", 0.0).getEntry();
    
  
 // private final CommandXboxController buttonBoard = new CommandXboxController(Ports.buttonBoard);
  //private final Score score = new Score(claw, swerve, arm, () -> 8);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

	public RobotContainer() {

		PathPlannerTrajectory score2Path = PathPlanner.loadPath("Score2", new PathConstraints(4, 3));
		PathPlannerTrajectory scoreAndBalancePath = PathPlanner.loadPath("ScoreAndBalance", new PathConstraints(4, 3));
		PathPlannerTrajectory scoreAndParkClosePath = PathPlanner.loadPath("ScoreAndParkClose", new PathConstraints(1, 2));
		PathPlannerTrajectory scoreAndParkLongPath = PathPlanner.loadPath("ScoreAndParkLong", new PathConstraints(1, 2));
		PathPlannerTrajectory score2FeederPath = PathPlanner.loadPath("Score2Feeder", new PathConstraints(4, 3));
		PathPlannerTrajectory balanceMiddlePath = PathPlanner.loadPath("BalanceMiddle", new PathConstraints(6, 4));
		PathPlannerTrajectory basicBalancePath = PathPlanner.loadPath("BasicBalance", new PathConstraints(2, 2));

		// This is just an example event map. It would be better to have a constant, global event map
		// in your code that will be used by all path following commands.
		HashMap<String, Command> score2Map = new HashMap<String, Command>();
		HashMap<String, Command> scoreAndBalanceMap = new HashMap<String, Command>();
		HashMap<String, Command> scoreAndParkCloseMap = new HashMap<String, Command>();
		HashMap<String, Command> scoreAndParkLongMap = new HashMap<String, Command>();
		HashMap<String, Command> score2FeederMap = new HashMap<String, Command>();
		HashMap<String, Command> balanceMiddleMap = new HashMap<String, Command>();
		HashMap<String, Command> basicBalanceMap = new HashMap<String, Command>();

	
		score2Map.put("score", new SequentialCommandGroup(Commands.runOnce(() -> claw.grab()), new HighAngles(arm, claw), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new WaitCommand(0.5), new HighAngles(arm, claw), new HighRetract(arm, claw)));
		score2Map.put("pickUp", new SequentialCommandGroup(new SetAngle(claw, arm), Commands.runOnce(() -> claw.unGrab()), new LineUpSwerveCone(claw, swerveSubsystem), new LineUpClaw(claw), new WaitCommand(0.25), new Attack(claw, arm), new WaitCommand(.5), new LineUpRetract(arm, claw)));
		score2Map.put("score2", new SequentialCommandGroup(new HighAngles(arm, claw), new LineUpConeRight(swerveSubsystem), new HighExtend(arm), new WaitCommand(.5), new HighDrop(arm, claw), new WaitCommand(0.5), new HighAngles(arm, claw), new HighRetract(arm, claw)));
		//eventMap.put("lineUp", lineUp);
		scoreAndBalanceMap.put("scoreCone", new SequentialCommandGroup(Commands.runOnce(() -> claw.grab()), new HighAngles(arm, claw), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new WaitCommand(0.5), new HighAngles(arm, claw), new HighRetract(arm, claw)));
		scoreAndBalanceMap.put("trainingWheels", new PrintCommand("I print things \n balls \n balls"));
		scoreAndBalanceMap.put("balance", new Balance(swerveSubsystem));

		scoreAndParkCloseMap.put("score", new SequentialCommandGroup(Commands.runOnce(() -> claw.grab()), new HighAngles(arm, claw), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new WaitCommand(0.5), new HighAngles(arm, claw), new HighRetract(arm, claw)));
		scoreAndParkLongMap.put("score", new SequentialCommandGroup(Commands.runOnce(() -> claw.grab()), new HighAngles(arm, claw), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new WaitCommand(0.5), new HighAngles(arm, claw), new HighRetract(arm, claw)));

		score2FeederMap.put("score", new SequentialCommandGroup(Commands.runOnce(() -> claw.grab()), new HighAngles(arm, claw), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new WaitCommand(0.5), new HighAngles(arm, claw), new HighRetract(arm, claw)));
		score2FeederMap.put("pickUp", new SequentialCommandGroup(new SetAngle(claw, arm), Commands.runOnce(() -> claw.unGrab()), new LineUpSwerveCone(claw, swerveSubsystem), new LineUpClaw(claw), new WaitCommand(0.25), new Attack(claw, arm), new WaitCommand(.5), new LineUpRetract(arm, claw)));
		score2FeederMap.put("score2", new SequentialCommandGroup(new HighAngles(arm, claw), new LineUpConeRight(swerveSubsystem), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new WaitCommand(0.5), new HighAngles(arm, claw), new HighRetract(arm, claw)));
		
		balanceMiddleMap.put("score", new SequentialCommandGroup(Commands.runOnce(() -> claw.grab()), new HighAngles(arm, claw), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new WaitCommand(0.5), new HighAngles(arm, claw), new HighRetract(arm, claw)));
		balanceMiddleMap.put("coolBalance", new PrintCommand("In progress"));

		basicBalanceMap.put("score", new SequentialCommandGroup(Commands.runOnce(() -> claw.grab()), new HighAngles(arm, claw), new HighExtend(arm),new WaitCommand(.5), new HighDrop(arm, claw), new WaitCommand(0.5), new HighAngles(arm, claw), new HighRetract(arm, claw)));
		basicBalanceMap.put("balance", new CoolBalance(swerveSubsystem));

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
		SwerveAutoBuilder scoreAndBalanceBuilder = new SwerveAutoBuilder(
			swerveSubsystem::getPose, // Pose2d supplier
			swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
			swerveSubsystem.kinematics, // SwerveDriveKinematics
			new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
			new PIDConstants(2.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
			swerveSubsystem::setAutoModuleStates, // Module states consumer used to output to the drive subsystem
			scoreAndBalanceMap,
			true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
			swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
		);
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
			new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
			new PIDConstants(2.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
			swerveSubsystem::setAutoModuleStates, // Module states consumer used to output to the drive subsystem
			score2FeederMap,
			true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
			swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
		);
		SwerveAutoBuilder basicBalanceBuilder = new SwerveAutoBuilder(
			swerveSubsystem::getPose, // Pose2d supplier
			swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
			swerveSubsystem.kinematics, // SwerveDriveKinematics
			new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
			new PIDConstants(1.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
			swerveSubsystem::setAutoModuleStates, // Module states consumer used to output to the drive subsystem
			basicBalanceMap,
			true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
			swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
		);
		
		score2 = score2Builder.fullAuto(score2Path);
		scoreAndBalance = scoreAndBalanceBuilder.fullAuto(scoreAndBalancePath);
		scoreAndParkClose = scoreAndParkCloseBuilder.fullAuto(scoreAndParkClosePath);
		scoreAndParkLong = scoreAndParkLongBuilder.fullAuto(scoreAndParkLongPath);
		score2Feeder = score2FeederBuilder.fullAuto(score2FeederPath);
		basicBalance = basicBalanceBuilder.fullAuto(basicBalancePath);
		justScore = new SequentialCommandGroup(Commands.runOnce(() -> claw.grab()), new HighAngles(arm, claw), new HighExtend(arm), new WaitCommand(.5), new HighDrop(arm, claw), new WaitCommand(0.5), new HighAngles(arm, claw), new HighRetract(arm, claw));

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
		//controller.y().whileTrue(Commands.run(() -> arm.setWinchPower(0.5)));
		//controller.y().whileTrue(Commands.startEnd(() -> arm.setWinchPower(1), () -> arm.setWinchPower(0)));
		//controller.a().whileTrue(Commands.startEnd(() -> arm.setWinchPower(-1), () -> arm.setWinchPower(0)));

		//controller.a().whileTrue(Commands.run(() -> arm.setWinchPower(-0.5)));
		//controller.y().whileTrue())
		controller.a().whileTrue(Commands.run(() -> arm.winchDownPos()));
		controller.b().whileTrue(Commands.run(() -> arm.extendPos()));
		controller.x().whileTrue(Commands.run(() -> arm.retractPos()));
		controller.leftBumper().whileTrue(new SequentialCommandGroup(Commands.runOnce(() -> claw.unGrab()), new WaitCommand(2), Commands.runOnce(() -> claw.stopGrab())));
		controller.rightBumper().onTrue(Commands.runOnce(() -> claw.toggleGrab()));
		controller.start().whileTrue(new SetAngle(claw, arm));
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

		buttonBoard.button(1).whileTrue(new LiftAngleHigh(claw, arm));
		buttonBoard.button(2).whileTrue(new LiftAngleMid(claw, arm));
		buttonBoard.button(3).whileTrue(new ExtendFar(claw, arm));
		buttonBoard.button(4).whileTrue(new ExtendShort(claw, arm));
		//buttonBoard.button(5).whileTrue(lineupMediumCube); LED
		//buttonBoard.button(6).whileTrue(lineupMediumConeRight); LED

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
			return scoreAndBalance;
		}
		else if(autoChoice.getDouble(0.0) == 2.0) {
			return scoreAndParkClose;
		}
		else if(autoChoice.getDouble(0.0) == 3.0) {
			return scoreAndParkLong;
		}
		else if(autoChoice.getDouble(0.0) == 4.0) {
			return score2;
		}
		else if(autoChoice.getDouble(0.0) == 5.0) {
			return score2Feeder;
		}
		else if(autoChoice.getDouble(0.0) == 6.0) {
			return basicBalance;
		}
		return justScore;
	
	}
}