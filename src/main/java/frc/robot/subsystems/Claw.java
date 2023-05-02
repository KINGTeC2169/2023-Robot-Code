// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Claw extends SubsystemBase {
	private static final TalonFX wristMotor = new TalonFX(Ports.wristMotor);
	private static final TalonSRX clawGrippers = new TalonSRX(Ports.clawGrippers);

	private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(Ports.wristEncoder);
	private double wristPos;


	

	private ShuffleboardTab tab = Shuffleboard.getTab("Arm");

	/** Creates a new ExampleSubsystem. */
	public Claw() {
		wristMotor.config_kP(0, 0.3);
		wristPos = wristMotor.getSelectedSensorPosition();
		wristMotor.configAllowableClosedloopError(0, 100);
		//tab.addDouble("Twist Encoder", () -> getTwistEncoder());
		tab.addDouble("Wrist Encoder", () -> getWristEncoder());
		tab.addDouble("Wrist Absolute", () -> getAbsoluteWrist());

		tab.addDouble("Wrist Current", () -> getWristCurrent());

		tab.addDouble("Grip Current", () -> getGrippersCurrent());
		tab.addBoolean("Is Grabbed", () -> isGrabbed()).withWidget(BuiltInWidgets.kBooleanBox);
		//twistEncoder.setPositionOffset(244/360);
		new Thread(() -> {
            try {
                Thread.sleep(2000);
                resetWristEncoder();

            } catch (Exception e) {
            }
        }).start();
		
	}

	@Override
	public void periodic() {
		boolean isIntaking;
		if(clawGrippers.getMotorOutputPercent() != 0) {
			isIntaking = true;
		} else {
			isIntaking = false;
		}
		SmartDashboard.putBoolean("Is Intaking", isIntaking);
	}
	public double getGrippersCurrent() {
		return clawGrippers.getSupplyCurrent();
	}
	public boolean isGrabbed() {
		return getGrippersCurrent() > 11.0;
	}

	public void wristUpPos() {
		wristPos = wristMotor.getSelectedSensorPosition() + 5000;
		if(wristPos > 0)
			wristPos = 0;
		wristMotor.set(ControlMode.Position, wristPos);
	}
	public void wristDownPos() {
		wristPos = wristMotor.getSelectedSensorPosition() - 5000;
		if(wristPos < -115 * 979.45)
			wristPos = -115 * 979.45;
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


  	public void moveWrist(double power) {
    	wristMotor.set(ControlMode.PercentOutput, power);
  	}
	public double getAbsoluteWrist() {
		return wristEncoder.getAbsolutePosition() * 360 - 273; //244
	}

	public void grab() {
		
		clawGrippers.set(ControlMode.PercentOutput, -1);
		
	}
	public void unGrab() {
		//public 
		
		clawGrippers.set(ControlMode.PercentOutput, 1);

	}
	public void stopGrab() {
		clawGrippers.set(ControlMode.PercentOutput, 0);
	} 
	public void toggleGrab() {
		if(Math.abs(clawGrippers.getMotorOutputPercent()) == 0 || clawGrippers.getMotorOutputPercent() == 1)
			clawGrippers.set(ControlMode.PercentOutput, -1);
		else
			clawGrippers.set(ControlMode.PercentOutput, 0);
		
	}

	public double getWristCurrent() {
		return wristMotor.getSupplyCurrent();
	}
	


	public double getWristEncoder() {
		return wristMotor.getSelectedSensorPosition() / 979.45;
	}


	public void resetWristEncoder() {
		wristMotor.setSelectedSensorPosition(getAbsoluteWrist() * 979.45);
	}


	public void stopWristPosition() {
		wristMotor.set(ControlMode.Position, wristMotor.getSelectedSensorPosition());
	}

	public static double getWristAngle() {
		return wristMotor.getSelectedSensorPosition() / 979.45;
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}