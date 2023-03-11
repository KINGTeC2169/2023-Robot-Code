// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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
	private double twistAngle = 0;
	private double twistOffset;
	
	

	private ShuffleboardTab tab = Shuffleboard.getTab("Arm");

	private Constraints twistLimits = new Constraints(0.5, 0.5);
	private GenericEntry twistP = tab.addPersistent("Twist P", 0.5).getEntry();
	//private ProfiledPIDController twistPID = new ProfiledPIDController(twistP.getDouble(0.5), 0.0, 0.0, twistLimits);
	/** Creates a new ExampleSubsystem. */
	public Claw() {
		wristMotor.config_kP(0, 0.3);
		clawTwist.config_kP(0, 0.5);
		wristPos = wristMotor.getSelectedSensorPosition();
		twistPos = clawTwist.getSelectedSensorPosition();
		wristMotor.configAllowableClosedloopError(0, 100);
		clawTwist.configClosedLoopPeakOutput(0, 0.5);
		//tab.addDouble("Twist Encoder", () -> getTwistEncoder());
		tab.addDouble("Wrist Encoder", () -> getWristEncoder());
		tab.addDouble("Twist Absolute", () -> getAbsoluteTwist());
		tab.addDouble("Wrist Absolute", () -> getAbsoluteWrist());

		tab.addDouble("Twist Current", () -> getTwistCurrent());
		tab.addDouble("Wrist Current", () -> getWristCurrent());

		tab.addDouble("Twist Relative", () -> getRelativeTwist());
		tab.addDouble("Absolute relative twist", () -> twistEncoder.get());
		//twistEncoder.setPositionOffset(244/360);
		new Thread(() -> {
            try {
                Thread.sleep(2000);
                resetWristEncoder();
				resetTwistEncoder();

            } catch (Exception e) {
            }
        }).start();
		
	}

	@Override
	public void periodic() {
		
		//clawTwist.set(ControlMode.PercentOutput, number);
	}

	public void twistUpPos() {
		clawTwist.config_kP(0, twistP.getDouble(0.5));

		twistAngle = getRelativeTwist() + 50;
		if (twistAngle > 100)
			twistAngle = 100;
		setTwistAngle(twistAngle);
		
	}
	public void twistDownPos() {
		clawTwist.config_kP(0, twistP.getDouble(0.5));

		twistAngle = getRelativeTwist() - 50;
		if (twistAngle < -100)
			twistAngle = -100;
		setTwistAngle(twistAngle);
	}
	public void wristUpPos() {
		wristPos = wristMotor.getSelectedSensorPosition() + 5000;
		if(wristPos > 0)
			wristPos = 0;
		wristMotor.set(ControlMode.Position, wristPos);
	}
	public void wristDownPos() {
		wristPos = wristMotor.getSelectedSensorPosition() - 5000;
		if(wristPos < -130 * 979.45)
			wristPos = -130 * 979.45;
		wristMotor.set(ControlMode.Position, wristPos);
	} 
	public double setWristAngle(double angle) {
		wristMotor.set(ControlMode.Position, angle * 979.45);
		return wristMotor.getClosedLoopError() / 979.45;
	}
	public double setWristAngleSlow(double angle) {
		wristMotor.configClosedLoopPeakOutput(0, 0.5);
		wristMotor.set(ControlMode.Position, angle * 979.45);
		wristMotor.configClosedLoopPeakOutput(0, .5);
		return wristMotor.getClosedLoopError() / 979.45;
	}
	public void wristStopPos() {
		wristMotor.set(ControlMode.Position, wristMotor.getSelectedSensorPosition());
	}
	public void twistStopPos() {
		//clawTwist.set(ControlMode.Position, clawTwist.getSelectedSensorPosition());
	}

  	public void twistClaw(double power) {
    	System.out.println("Turning: " + power);
    	clawTwist.set(ControlMode.PercentOutput, power);
  	}
  	public void moveWrist(double power) {
    	wristMotor.set(ControlMode.PercentOutput, power);
  	}
	public double getAbsoluteWrist() {
		return wristEncoder.getAbsolutePosition() * 360 - 244;
	}
	public double getAbsoluteTwist() {
		return twistEncoder.getAbsolutePosition() * 360 - 130;
	}
	public double getRelativeTwist() {
		return clawTwist.getSelectedSensorPosition() / 8192 * 360;
	}

	public void grab() {
		grabber.set(Value.kForward);
		
	}
	public void unGrab() {
		grabber.set(Value.kReverse);
	}
	public void setGrab(Boolean isGrab) {
		//grabber.set(Value.kReverse);
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
		return wristMotor.getSelectedSensorPosition() / 979.45;
	}

	public double getTwistEncoder() {
		return clawTwist.getSelectedSensorPosition() / 8192;
	}

	public void resetWristEncoder() {
		wristMotor.setSelectedSensorPosition(getAbsoluteWrist() * 979.45);
	}

	public void resetTwistEncoder() {
		//twistOffset = getAbsoluteTwist();
		clawTwist.setSelectedSensorPosition(getAbsoluteTwist() / 360 * 8192);
	}

	public double setTwistAngle(double angle) {
		if (angle > 100)
			angle = 100;
		if (angle < -100)
			angle = -100;
		clawTwist.config_kP(0, twistP.getDouble(0.5));

		//clawTwist.set(ControlMode.Position, angle);
		clawTwist.set(ControlMode.Position, angle / 360 * 8192);
		return clawTwist.getClosedLoopError();
	}
	public double setTwistAngle(double angle, double kP) {
		if (angle > 100)
			angle = 100;
		if (angle < -100)
			angle = -100;
		clawTwist.config_kP(0, kP);
		clawTwist.set(ControlMode.Position, angle / 360 * 8192);
		return clawTwist.getClosedLoopError();
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}