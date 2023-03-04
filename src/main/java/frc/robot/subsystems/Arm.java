package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Arm extends SubsystemBase {

    private final TalonFX elevatorMotor = new TalonFX(Ports.elevatorMotor);
    private final TalonFX winchMotor = new TalonFX(Ports.winchMotor);
    private double winchPos;
    private double elevatorPos;


    private final double ELEVATOR_UPPER_LIMIT = 320000;
    private final double ELEVATOR_LOWER_LIMIT = 0;
    private final double WINCH_UPPER_LIMIT = 200000;
    private final double WINCH_LOWER_LIMIT = 0;
    private final double ELEVATOR_SPEED = 20000;
    private final double WINCH_SPEED = 30000;

    private ShuffleboardTab tab = Shuffleboard.getTab("Arm");
    private GenericEntry elevatorUpperLimit = tab.addPersistent("Elevator Upper Limit", ELEVATOR_UPPER_LIMIT).withSize(2, 1).withPosition(4, 0).getEntry();
    private GenericEntry elevatorLowerLimit = tab.addPersistent("Elevator Lower Limit", ELEVATOR_LOWER_LIMIT).withSize(2, 1).withPosition(4, 1).getEntry();
    private GenericEntry winchUpperLimit = tab.addPersistent("Winch Upper Limit", WINCH_UPPER_LIMIT).withSize(2, 1).withPosition(2, 0).getEntry();
    private GenericEntry winchLowerLimit = tab.addPersistent("Winch Lower Limit", WINCH_LOWER_LIMIT).withSize(2, 1).withPosition(2, 1).getEntry();

    private GenericEntry elevatorSpeed = tab.addPersistent("Elevator Speed", ELEVATOR_SPEED).withSize(2, 1).withPosition(4, 2).getEntry();
    private GenericEntry winchSpeed = tab.addPersistent("Winch Speed", WINCH_SPEED).withSize(2, 1).withPosition(2, 2).getEntry();

    
    /**
     * Creates a new ExampleSubsystem.
     */
    public Arm() {

        ShuffleboardLayout currents = tab.getLayout("Arm Currents", BuiltInLayouts.kGrid).withSize(2, 1).withProperties(Map.of("Number of rows", 1)).withPosition(0, 0);

        elevatorMotor.config_kP(0, 0.1);
        winchMotor.config_kP(0, 0.1);
        winchPos = winchMotor.getSelectedSensorPosition(); 
        tab.addDouble("Elevator Position", () -> getElevatorEncoder()).withPosition(7, 0);

        tab.addDouble("Winch Position", () -> getLiftAngle()).withPosition(7, 1);

        currents.addDouble("Winch Current", () -> getWinchCurrent()).withWidget(BuiltInWidgets.kVoltageView);
        currents.addDouble("Elevator Current", () -> getElevatorCurrent()).withWidget(BuiltInWidgets.kVoltageView);

        tab.add("Reset Elevator Position", Commands.runOnce(() -> resetElevatorEncoder())).withPosition(8, 0).withSize(2, 1);
        tab.add("Reset Winch Position",Commands.runOnce(() -> resetWinchEncoder())).withPosition(8, 1).withSize(2, 1);
    }

    @Override
    public void periodic() {
    }

    /**Extends arm by elevator speed value from shuffleboard. ControlMode.Position */
    public void extendPos() {
        double pos = elevatorMotor.getSelectedSensorPosition();
        double speed = elevatorSpeed.getDouble(ELEVATOR_SPEED);

        if(pos + speed < elevatorUpperLimit.getDouble(ELEVATOR_UPPER_LIMIT)) {
            elevatorPos = pos + speed;
            elevatorMotor.set(ControlMode.Position, elevatorPos);
        }
    }
    /**Retracts arm by elevator speed value from shuffleboard. ControlMode.Position */
    public void retractPos() {
        double pos = elevatorMotor.getSelectedSensorPosition();
        double speed = elevatorSpeed.getDouble(ELEVATOR_SPEED);

        if(pos - speed > elevatorLowerLimit.getDouble(ELEVATOR_LOWER_LIMIT)) {
            elevatorPos = pos - speed;
            elevatorMotor.set(ControlMode.Position, elevatorPos);
        }
    }
    /**Lifts arm by winch speed value from shuffleboard. ControlMode.Position */
    public void winchUpPos() {
        double pos = winchMotor.getSelectedSensorPosition();
        double speed = winchSpeed.getDouble(WINCH_SPEED);

        if(pos + speed < winchUpperLimit.getDouble(WINCH_UPPER_LIMIT)) {
            winchPos = pos + speed;
            winchMotor.set(ControlMode.Position, winchPos);
        }
        
    }
    /**Lowers arm by winch speed value from shuffleboard. ControlMode.Position */
    public void winchDownPos() {
        double pos = winchMotor.getSelectedSensorPosition();
        double speed = winchSpeed.getDouble(WINCH_SPEED);

        if(pos - speed > winchLowerLimit.getDouble(WINCH_LOWER_LIMIT)) {
            winchPos = pos - speed;
            winchMotor.set(ControlMode.Position, winchPos);
        }
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

    public double getElevatorCurrent() {
        return elevatorMotor.getSupplyCurrent();
    }

    
    public void resetWinchEncoder() {
        System.out.println("reset winch");
        winchMotor.setSelectedSensorPosition(0);
        winchMotor.set(ControlMode.Position, 0);
    }
    public void resetElevatorEncoder() {
        System.out.println("reset elevator");
        elevatorMotor.setSelectedSensorPosition(0);
        elevatorMotor.set(ControlMode.Position, 0);
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
    public void winchStopPos() {
        winchMotor.set(ControlMode.Position, winchMotor.getSelectedSensorPosition());
    }
    public double getWinchCurrent() {
        return winchMotor.getSupplyCurrent();
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