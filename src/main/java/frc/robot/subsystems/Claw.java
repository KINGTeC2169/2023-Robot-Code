// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Claw extends SubsystemBase {
  private final TalonFX wrist = new TalonFX(Ports.wristMotor);
  //private final PIDController pid = new PIDController(0.5, 0, 0);
  /** Creates a new ExampleSubsystem. */
  public Claw() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void turnWrist(double power) {
    System.out.println("Turning: " + power);
    wrist.set(ControlMode.PercentOutput, power);
  }

  public double getEnconder() {
    return wrist.getSelectedSensorPosition();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
