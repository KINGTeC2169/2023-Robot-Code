package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Arm extends SubsystemBase {

    private final TalonFX elevatorMotor = new TalonFX(Ports.elevatorMotor);
    private final TalonFX winchMotor = new TalonFX(Ports.winchMotor);
    private final PIDController winchPid = new PIDController(0.5, 0, 0);
    private final PIDController elevatorPid = new PIDController(0.5, 0, 0);

    /**
     * Creates a new ExampleSubsystem.
     */
    public Arm() {
        winchPid.setTolerance(3);
        elevatorPid.setTolerance(3);
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
    public void winchDown() {
        winchMotor.set(ControlMode.PercentOutput, -0.3);
    }
    public void winchStop() {
        winchMotor.set(ControlMode.PercentOutput, 0);
    }
    public double setArmAngle(double angle) {
        winchMotor.set(ControlMode.Position, angle);
        return winchMotor.getClosedLoopError();
    }


    public double getLiftAngle() {
        return winchMotor.getSelectedSensorPosition();//TODO: convert to degrees
        
    }

    




}
