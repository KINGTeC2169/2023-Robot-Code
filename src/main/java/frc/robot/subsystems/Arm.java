package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Arm extends SubsystemBase {

    private final TalonFX elevatorMotor = new TalonFX(Ports.elevatorMotor);
    private final TalonFX winchMotor = new TalonFX(Ports.winchMotor);
    //private final PIDController pid = new PIDController(0.5, 0, 0);

    /**
     * Creates a new ExampleSubsystem.
     */
    public Arm() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Elevator Encoder", getElevatorEncoder());
        SmartDashboard.putNumber("Lift Encoder", getLiftEncoder());
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

    public void setWinch(double power) {
        winchMotor.set(ControlMode.PercentOutput, power);
    }
    public void winchUp() {
        System.out.println("Balls in my mouth");
        winchMotor.set(ControlMode.PercentOutput, 0.3);
    }
    public void winchDown() {
        winchMotor.set(ControlMode.PercentOutput, -0.3);
    }
    public void winchStop() {
        winchMotor.set(ControlMode.PercentOutput, 0);
    }


    public double getLiftEncoder() {
        return winchMotor.getSelectedSensorPosition();
    }

    




}
