package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Arm extends SubsystemBase {

    private static final TalonFX elevatorMotor = new TalonFX(Ports.elevatorMotor);
    private final TalonFX winchMotor = new TalonFX(Ports.winchMotor);
    private final DutyCycleEncoder angleEncoder = new DutyCycleEncoder(Ports.armEncoder);
    private double winchPos;
    private double elevatorPos;
    


    private final double ELEVATOR_UPPER_LIMIT = 81200;
    private final double ELEVATOR_LOWER_LIMIT = 0;
    private final double WINCH_UPPER_LIMIT = 70;
    private final double WINCH_LOWER_LIMIT = 12.5;
    private final double ELEVATOR_SPEED = 10000;
    private final double WINCH_SPEED = 5;
    private final double lowLimit = 80000;
    //private boolean wristWillBreak;

    private ShuffleboardTab tab = Shuffleboard.getTab("Arm");
    //private GenericEntry elevatorUpperLimit = tab.addPersistent("Elevator Upper Limit", ELEVATOR_UPPER_LIMIT).withSize(2, 1).withPosition(4, 0).getEntry();
    //private GenericEntry elevatorLowerLimit = tab.addPersistent("Elevator Lower Limit", ELEVATOR_LOWER_LIMIT).withSize(2, 1).withPosition(4, 1).getEntry();
    //private GenericEntry winchUpperLimit = tab.addPersistent("Winch Upper Limit", WINCH_UPPER_LIMIT).withSize(2, 1).withPosition(2, 0).getEntry();
    //private GenericEntry winchLowerLimit = tab.addPersistent("Winch Lower Limit", WINCH_LOWER_LIMIT).withSize(2, 1).withPosition(2, 1).getEntry();

    //private GenericEntry elevatorSpeed = tab.addPersistent("Elevator Speed", ELEVATOR_SPEED).withSize(2, 1).withPosition(4, 2).getEntry();
    //private GenericEntry winchSpeed = tab.addPersistent("Winch Speed", WINCH_SPEED).withSize(2, 1).withPosition(2, 2).getEntry();

    
    /**
     * Creates a new ExampleSubsystem.
     */
    public Arm() {

        ShuffleboardLayout currents = tab.getLayout("Arm Currents", BuiltInLayouts.kGrid).withSize(2, 1).withProperties(Map.of("Number of rows", 1)).withPosition(0, 0);

        elevatorMotor.config_kP(0, 0.1);
        elevatorMotor.config_kF(0, 0.1); //figure this out :(
        elevatorMotor.configMotionCruiseVelocity(10, 0);
        winchMotor.config_kP(0, 0.1);
        winchMotor.configMotionAcceleration(10000000);
        
        winchPos = winchMotor.getSelectedSensorPosition(); 
        tab.addDouble("Elevator Position", () -> getElevatorEncoder()).withPosition(7, 0);

        tab.addDouble("Winch Position", () -> getLiftAngle()).withPosition(7, 1);
        tab.addDouble("Absolute Angle", () -> getAngleAbsolute()).withPosition(7, 2);

        currents.addDouble("Winch Current", () -> getWinchCurrent()).withWidget(BuiltInWidgets.kVoltageView);
        currents.addDouble("Elevator Current", () -> getElevatorCurrent()).withWidget(BuiltInWidgets.kVoltageView);

        tab.add("Reset Elevator Position", Commands.runOnce(() -> resetElevatorEncoder())).withPosition(8, 0).withSize(2, 1);
        tab.add("Reset Winch Position", Commands.runOnce(() -> resetWinchEncoder())).withPosition(8, 1).withSize(2, 1);
        //tab.addBoolean("Wrist will break", () -> wristWillBreak);

        resetWinchEncoder();
    }

    @Override
    public void periodic() {
    }


    /**Extends arm by elevator speed value from shuffleboard. ControlMode.Position */
    public void extendPos() {
        double pos = elevatorMotor.getSelectedSensorPosition();
        double speed = ELEVATOR_SPEED;

        if(pos + speed < ELEVATOR_UPPER_LIMIT) {
            elevatorPos = pos + speed;
            elevatorMotor.set(ControlMode.MotionMagic, elevatorPos);
        }
    }
    /**Retracts arm by elevator speed value from shuffleboard. ControlMode.Position */
    public void retractPos() {
        double pos = elevatorMotor.getSelectedSensorPosition();
        double speed = ELEVATOR_SPEED;

        if(pos - speed > ELEVATOR_LOWER_LIMIT) {
                    elevatorPos = pos - speed;
                    elevatorMotor.set(ControlMode.MotionMagic, elevatorPos);
        }
        

    }

    /**Hopefully do not need to use these :( */
    public void extendPosLimitless() {
        double pos = elevatorMotor.getSelectedSensorPosition();
        double speed = ELEVATOR_SPEED;

        elevatorPos = pos + speed;
        elevatorMotor.set(ControlMode.MotionMagic, elevatorPos);
    
    }
    public void retractPosLimitless() {
        double pos = elevatorMotor.getSelectedSensorPosition();
        double speed = ELEVATOR_SPEED;

      
        elevatorPos = pos - speed;
        elevatorMotor.set(ControlMode.MotionMagic, elevatorPos);

    }
    public void setWinch(double angle) {
        if(angle < WINCH_UPPER_LIMIT && angle > WINCH_LOWER_LIMIT)
            //winchMotor.set(ControlMode.Position, angle * 7010.837);
            winchMotor.set(ControlMode.MotionMagic, angle * 7010.837, DemandType.Neutral, 1);
    }
    /**Lifts arm by winch speed value from shuffleboard. ControlMode.Position */
    public void winchUpPos() {
        double pos = getLiftAngle();
        double speed = WINCH_SPEED;

        if(pos + speed < WINCH_UPPER_LIMIT) {
            winchPos = pos + speed;
            setWinch(winchPos);
        }
        else {
            setWinch(WINCH_UPPER_LIMIT);
        }
        
    }
    /**Lowers arm by winch speed value from shuffleboard. ControlMode.Position */
    public void winchDownPos() {
        double pos = getLiftAngle();
        double speed = WINCH_SPEED;

        if(pos - speed > WINCH_LOWER_LIMIT) {
            winchPos = pos - speed;
            setWinch(winchPos);
        }
        else {
            setWinch(WINCH_LOWER_LIMIT);
        }
    }

    public double getElevatorEncoder() {
        return elevatorMotor.getSelectedSensorPosition();
    }
    public double setElevatorPosition(double pos) {
        if(pos > ELEVATOR_LOWER_LIMIT && pos < ELEVATOR_UPPER_LIMIT)
            elevatorMotor.set(ControlMode.MotionMagic, pos);
        return elevatorMotor.getClosedLoopError();
    }

    public void stopElevatorPosition() {
        elevatorMotor.set(ControlMode.MotionMagic, elevatorMotor.getSelectedSensorPosition());
    }

    public void stopWinchPosition() {
        winchMotor.set(ControlMode.Position, winchMotor.getSelectedSensorPosition());
    }

    public double getElevatorCurrent() {
        return elevatorMotor.getSupplyCurrent();
    }

    
    public void zeroWinchEncoder() {
        System.out.println("reset winch");
        winchMotor.setSelectedSensorPosition(0);
        winchMotor.set(ControlMode.Position, 0);
    }
    public void resetElevatorEncoder() {
        System.out.println("reset elevator");
        elevatorMotor.setSelectedSensorPosition(0);
        elevatorMotor.set(ControlMode.Position, 0);
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

    public double getAngleAbsolute() {
        return angleEncoder.getAbsolutePosition() * 360 - 267;
    }

    public double getLiftAngle() {
        return winchMotor.getSelectedSensorPosition() / 7010.837;
        
    }

    //TODO: check if this is right, I think it is but I dont know why you got rid of it
    public double setArmAngle(double degrees) {
        winchMotor.set(ControlMode.Position, degrees * 7010.837);
        return winchMotor.getClosedLoopError();
    }

    public void resetWinchEncoder() {
        winchMotor.setSelectedSensorPosition(getAngleAbsolute() * 7010.837);
    }

    public static double getElevator() {
        return elevatorMotor.getSelectedSensorPosition();
    }

    




}