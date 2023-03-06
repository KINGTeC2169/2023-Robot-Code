// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Claw extends SubsystemBase {
	private final TalonFX wristMotor = new TalonFX(Ports.wristMotor);
	private final TalonSRX clawTwist = new TalonSRX(Ports.clawTwist);
	private final DoubleSolenoid grabber = new DoubleSolenoid(PneumaticsModuleType.REVPH, Ports.grabberOne, Ports.grabberTwo);
	private final DutyCycleEncoder twistEncoder = new DutyCycleEncoder(Ports.twistEncoder);
	private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(Ports.wristEncoder);
	private double wristPos;
	private double twistPos;

	private ShuffleboardTab tab = Shuffleboard.getTab("Arm");

	/** Creates a new ExampleSubsystem. */
	public Claw() {

		wristMotor.config_kP(0, 0.3);
		clawTwist.config_kP(0, 0.5);
		wristPos = wristMotor.getSelectedSensorPosition();
		twistPos = clawTwist.getSelectedSensorPosition();
		wristMotor.configAllowableClosedloopError(0, 100);

		//tab.addDouble("Twist Encoder", () -> getTwistEncoder());
		tab.addDouble("Wrist Encoder", () -> getWristEncoder());
		tab.addDouble("Twist Absolute", () -> twistEncoder.getAbsolutePosition());
		tab.addDouble("Wrist Absolute", () -> wristEncoder.getAbsolutePosition());

		tab.addDouble("Twist Current", () -> getTwistCurrent());
		tab.addDouble("Wrist Current", () -> getWristCurrent());

	}

	@Override
	public void periodic() {
	}

	public void twistUpPos() {
		twistPos += 500;
		clawTwist.set(ControlMode.Position, twistPos);
	}
	public void twistDownPos() {
		twistPos -= 500;
		clawTwist.set(ControlMode.Position, twistPos);
	}
	public void wristUpPos() {
		wristPos = wristMotor.getSelectedSensorPosition() + 3000;
		wristMotor.set(ControlMode.Position, wristPos);
	}
	public void wristDownPos() {
		wristPos = wristMotor.getSelectedSensorPosition() - 3000;
		wristMotor.set(ControlMode.Position, wristPos);
	}
	public void wristStopPos() {
		wristMotor.set(ControlMode.Position, wristMotor.getSelectedSensorPosition());
	}
	public void twistStopPos() {
		clawTwist.set(ControlMode.Position, clawTwist.getSelectedSensorPosition());
	}

  	public void twistClaw(double power) {
    	System.out.println("Turning: " + power);
    	clawTwist.set(ControlMode.PercentOutput, power);
  	}
  	public void moveWrist(double power) {
    	wristMotor.set(ControlMode.PercentOutput, power);
  	}

	public void grab() {
		//grabber.set(true);
		
	}
	public void unGrab() {
		//grabber.set(false);
	}
	public void setGrab(Boolean isGrab) {
		//grabber.set(isGrab);
	} 
	public void toggleGrab() {
		if(grabber.get() == Value.kOff)
			grabber.set(Value.kForward);
		else
			grabber.toggle();
		
	}

	public void twistMax() {
		clawTwist.set(ControlMode.PercentOutput, 1);
	}

	public double getWristCurrent() {
		return wristMotor.getSupplyCurrent();
	}
	
	public double getTwistCurrent() {
		return clawTwist.getSupplyCurrent();
	}

	public double getWristEncoder() {
		return wristMotor.getSelectedSensorPosition();
	}

	public double getTwistEncoder() {
		return clawTwist.getSelectedSensorPosition();
	}

	public void resetWristEncoder() {
		wristMotor.setSelectedSensorPosition(0);
	}

	public void resetTwistEncoder() {
		clawTwist.setSelectedSensorPosition(0);
	}

	public double setTwistAngle(double angle) {
		//TODO convert angle to encoder ticks
		clawTwist.set(ControlMode.Position, angle);
		return clawTwist.getClosedLoopError();
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}