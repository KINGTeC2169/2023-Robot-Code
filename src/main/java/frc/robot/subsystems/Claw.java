// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Claw extends SubsystemBase {
  private final TalonFX wristMotor = new TalonFX(Ports.wristMotor);
  private final TalonSRX clawTwist = new TalonSRX(Ports.clawTwist);
  private final Solenoid grabber = new Solenoid(PneumaticsModuleType.REVPH, Ports.grabber);
  //private final PIDController pid = new PIDController(0.5, 0, 0);
  /** Creates a new ExampleSubsystem. */
  public Claw() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Encoder", getWristEncoder());
    SmartDashboard.putNumber("ClawTwist", getTwistEncoder());
  }

  public void twistClaw(double power) {
    System.out.println("Turning: " + power);
    clawTwist.set(ControlMode.PercentOutput, power);
  }
  public void moveWrist(double power) {
    wristMotor.set(ControlMode.PercentOutput, power);
  }

  public void grab() {
    grabber.set(true);
  }
  public void unGrab() {
    grabber.set(false);
  }
  public void setGrab(Boolean isGrab) {
    grabber.set(isGrab);
  } 

  public double getWristEncoder() {
    return wristMotor.getSelectedSensorPosition();
  }

  public double getTwistEncoder() {
    return clawTwist.getSelectedSensorPosition();
  }

  public void resestWristEncoder() {
    wristMotor.setSelectedSensorPosition(0);
  }

  public void resetTwistEncoder() {
    clawTwist.setSelectedSensorPosition(0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
