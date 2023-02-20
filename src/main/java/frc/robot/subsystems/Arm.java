package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Arm extends SubsystemBase {

    private final TalonFX elevatorMotor = new TalonFX(Ports.elevatorMotor);
    private final TalonFX winchMotor = new TalonFX(Ports.winchMotor);
    private double winchPos;
    private double elevatorPos;

    /**
     * Creates a new ExampleSubsystem.
     */
    public Arm() {
        elevatorMotor.config_kP(0, 0.1);
        winchMotor.config_kP(0, 0.1);
        winchPos = winchMotor.getSelectedSensorPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Elevator Encoder", getElevatorEncoder());
        SmartDashboard.putNumber("Lift Encoder", getLiftAngle());
    }

    public void extend(double power) {
        elevatorMotor.set(ControlMode.PercentOutput, power);
    }

    public void retract(double power) {
        elevatorMotor.set(ControlMode.PercentOutput, -power);
    }
    public void extendPos() {
        elevatorPos = elevatorMotor.getSelectedSensorPosition() + 20000;
        elevatorMotor.set(ControlMode.Position, elevatorPos);
    }
    public void retractPos() {
        elevatorPos = elevatorMotor.getSelectedSensorPosition() - 20000;
        elevatorMotor.set(ControlMode.Position, elevatorPos);
    }


    public void setElevatorPower(double power) {
        elevatorMotor.set(ControlMode.PercentOutput, power);
    }

    public double getElevatorEncoder() {
        return elevatorMotor.getSelectedSensorPosition();
    }
    public double setElevatorPosition(double inches) {
        winchMotor.set(ControlMode.Position, inches);
        return winchMotor.getClosedLoopError();
    }

    public void setWinch(double power) {
        winchMotor.set(ControlMode.PercentOutput, power);
    }
    public void winchUp() {
        winchMotor.set(ControlMode.PercentOutput, 0.3);
    }
    public void winchUpPos() {
        winchPos = winchMotor.getSelectedSensorPosition() + 30000;
        winchMotor.set(ControlMode.Position, winchPos);
    }
    public void winchDownPos() {
        winchPos = winchMotor.getSelectedSensorPosition() - 30000;
        winchMotor.set(ControlMode.Position, winchPos);
    }
    public void winchDown() {
        winchMotor.set(ControlMode.PercentOutput, -0.3);
    }
    public void winchStop() {
        winchMotor.set(ControlMode.PercentOutput, 0);
    }
    public void winchStopPos() {
        winchMotor.set(ControlMode.Position, winchMotor.getSelectedSensorPosition());
    }
    public void elevatorStopPos() {
        elevatorMotor.set(ControlMode.Position, elevatorMotor.getSelectedSensorPosition());
    }
    public double setArmAngle(double angle) {
        //TODO: convert angle into encoder ticks
        winchMotor.set(ControlMode.Position, angle);
        return winchMotor.getClosedLoopError();
    }



    public double getLiftAngle() {
        return winchMotor.getSelectedSensorPosition();//TODO: convert to degrees
        
    }

    




}
