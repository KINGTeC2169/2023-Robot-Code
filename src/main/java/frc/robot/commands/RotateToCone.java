// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.Motors;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.NetworkTables;

import java.util.function.Supplier;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RotateToCone extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Claw claw;


  private final Timer time;
  private double angle;
  private boolean isAngle;
  
  private final PIDController pid;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RotateToCone(Claw claw) {
    this.claw = claw;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
    pid = new PIDController(0.5, 0, 0);
    time = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.start();
    claw.resestEncoder();
    angle = NetworkTables.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //angle = NetworkTables.getAngle();

    //  x/2048 * 360


    double currentAngle = claw.getEnconder() / Motors.TalonFXCPR * 360;

    double power = pid.calculate(currentAngle, angle);

    System.out.println("Current angle: " + currentAngle + "\tAngle: " + power + "\tPower: " + power);

    //claw.turnWrist(clawTwist.get());


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //time.getUsClock()() > 1 && Math.abs(NetworkTable.getAngle()) < 5?
    //return isAngle;
    return false;
  }
}
