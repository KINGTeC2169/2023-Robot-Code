// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.NetworkTables;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RotateToCone extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Claw m_claw;

  private final Supplier<Double> clawTwist;
  private final Timer time;
  private double angle;
  
  private final PIDController pid;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RotateToCone(Claw claw, Supplier<Double> twist) {
    m_claw = claw;

    clawTwist = twist;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
    pid = new PIDController(0.5, 0, 0);
    time = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = NetworkTables.getAngle();

    double power = pid.calculate(NetworkTables.getAngle(), 45);
    System.out.println("Power: " + power);



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //time.get() > 1 && 
    return false;
  }
}
