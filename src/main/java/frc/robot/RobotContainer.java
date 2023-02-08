// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.Ports;
import frc.robot.commands.GetCubone;
import frc.robot.commands.Score;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CuboneManager;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  //private final Claw claw = new Claw();
  //private final Arm arm = new Arm();
  //private final CuboneManager cuboneManager = new CuboneManager();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  //private final NetworkTables tables = new NetworkTables();

  //private final XboxController controller = new XboxController(Ports.controller);
  //private final GetCubone rotateToCone = new GetCubone(claw, swerve, arm);
  private final CommandXboxController controller = new CommandXboxController(Ports.controller);
 // private final CommandXboxController buttonBoard = new CommandXboxController(Ports.buttonBoard);
  //private final Score score = new Score(claw, swerve, arm, () -> 8);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //m_claw.setDefaultCommand(rotateToCone);
    // Configure the button bindings

    swerveSubsystem.setDefaultCommand(new SwerveCommand(
          swerveSubsystem,
          () -> controller.getLeftY(),
          //() -> driverJoystick.getY(),
          //() -> driverJoystick.getX(),
          () -> controller.getLeftX(),
          () -> controller.getRightX()));
          //() -> driverJoystick2.getX(),
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
    //controller.y().whileTrue(Commands.startEnd(() -> arm.winchUp(), () -> arm.winchStop(), arm));
    //controller.a().whileTrue(Commands.startEnd(() -> arm.winchUp(), () -> arm.winchStop(), arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
