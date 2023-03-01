package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Arm extends SubsystemBase {

    private final TalonFX elevatorMotor = new TalonFX(Ports.elevatorMotor);
    private final TalonFX winchMotor = new TalonFX(Ports.winchMotor);
    private double winchPos;
    private double elevatorPos;


    private final double ELEVATOR_UPPER_LIMIT = 200000;
    private final double ELEVATOR_LOWER_LIMIT = 0;
    private final double WINCH_UPPER_LIMIT = 200000;
    private final double WINCH_LOWER_LIMIT = 0;
    private final double ELEVATOR_SPEED = 20000;
    private final double WINCH_SPEED = 30000;

    private ShuffleboardTab tab = Shuffleboard.getTab("Arm");
    private GenericEntry elevatorUpperLimit = tab.addPersistent("Elevator Upper Limit", ELEVATOR_UPPER_LIMIT).getEntry();
    private GenericEntry elevatorLowerLimit = tab.addPersistent("Elevator Lower Limit", ELEVATOR_LOWER_LIMIT).getEntry();
    private GenericEntry winchUpperLimit = tab.addPersistent("Winch Upper Limit", WINCH_UPPER_LIMIT).getEntry();
    private GenericEntry winchLowerLimit = tab.addPersistent("Winch Lower Limit", WINCH_LOWER_LIMIT).getEntry();

    private GenericEntry elevatorSpeed = tab.addPersistent("Elevator Speed", ELEVATOR_SPEED).getEntry();
    private GenericEntry winchSpeed = tab.addPersistent("Winch Speed", WINCH_SPEED).getEntry();

    
    /**
     * Creates a new ExampleSubsystem.
     */
    public Arm() {
        elevatorMotor.config_kP(0, 0.1);
        winchMotor.config_kP(0, 0.1);
        winchPos = winchMotor.getSelectedSensorPosition(); 
        tab.addDouble("Elevator Position", () -> getElevatorEncoder());
        tab.addDouble("Winch Position", () -> getLiftAngle());
        tab.addDouble("Winch Current", () -> getWinchCurrent());
        tab.addDouble("Elevator Current", () -> getElevatorCurrent());
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
    public void setWinchPos(double degrees) {
        //TODO: make this work
        //winchMotor.set(ControlMode.Position, degrees /*TODO:constant*/);

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
