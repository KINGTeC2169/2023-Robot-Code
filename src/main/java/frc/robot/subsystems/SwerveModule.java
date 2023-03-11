package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.ModuleConstants.*;

public class SwerveModule {
    private TalonFX driveMotor;
    private CANSparkMax turnMotor;
    private CANCoder absoluteEncoder;

    private final RelativeEncoder turnEncoder;

    private double wantedSpeed;

    private PIDController drivePID;
    private PIDController turningPID;
    private CANCoderConfiguration config = new CANCoderConfiguration();

    public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorReversed, 
                boolean turnMotorReversed, int canCoderID, double absoluteOffset, 
                boolean isCancoderReversed) {

        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        driveMotor.configFactoryDefault();
        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        config.magnetOffsetDegrees = Units.radiansToDegrees(absoluteOffset);
        config.sensorCoefficient = 2 * Math.PI / 4096.0;
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        config.sensorDirection = isCancoderReversed;
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;

        absoluteEncoder = new CANCoder(canCoderID);
        absoluteEncoder.configAllSettings(config);


        turnEncoder = turnMotor.getEncoder();
        turnEncoder.setPositionConversionFactor(turnEncoderToRadian);
        turnEncoder.setVelocityConversionFactor(turnEncoderRPMToRadPerSec);
        //driveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        /* 
        driveEncoder.setPositionConversionFactor(driveEncoderToMeter);
        driveEncoder.setVelocityConversionFactor(driveEncoderRPMToMeterPerSec);
        */

        //Creating and configuring PID controllers
        turningPID = new PIDController(PTurn, 0, 0);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);

        driveMotor.config_kP(0, 0.1);
        
        //drivePID = new PIDController(PDrive, 0, 0);


        resetEncoders();

    }

    public double getDrivePosition() {
        //return driveMotor.getSelectedSensorPosition() * driveEncoderToMeter;
        return driveMotor.getSelectedSensorPosition() * (0.32 / 13824);
    }

    /**Returns position of turn encoder in radians. Counterclockwise is positive, accumulates. */
    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }
    public Rotation2d getRotation2d() {
        //return Rotation2d.fromDegrees(getTurnPosition());
        return Rotation2d.fromRadians(getTurnPosition());
    }

    
    public double getDriveVelocity() {
        //return driveMotor.getSelectedSensorVelocity() * driveEncoderRPMToMeterPerSec;
        return driveMotor.getSelectedSensorVelocity();
    }

    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
        //>return turnMotor.getSelectedSensorVelocity();
    }

    public double getAbsoluteTurnPosition() {
        return absoluteEncoder.getAbsolutePosition();
    }

    public double getDriveCurrent() {
        return driveMotor.getSupplyCurrent();
    }

    public double getTurnCurrent() {
        return turnMotor.getOutputCurrent();
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turnEncoder.setPosition(getAbsoluteTurnPosition());
        System.out.println("RESETTING ENCODERS \nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS");
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), getRotation2d());
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        if(state.speedMetersPerSecond > 0) {
            //driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / maxSpeed);
        wantedSpeed = (((state.speedMetersPerSecond / maxSpeed) * 0.94) + 0.06);
        //driveMotor.set(ControlMode.Velocity, wantedSpeed * 3 / 2);
        // I LOVE ALIVEBAND  
        driveMotor.set(ControlMode.PercentOutput, ((state.speedMetersPerSecond / maxSpeed) * 0.94) + 0.06);
            
        } else {
            //driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / maxSpeed);
        wantedSpeed = (((state.speedMetersPerSecond / maxSpeed) * 0.94) - 0.06);
        //driveMotor.set(ControlMode.Velocity, wantedSpeed * 3 / 2);
        // I LOVE ALIVEBAND  
        driveMotor.set(ControlMode.PercentOutput, ((state.speedMetersPerSecond / maxSpeed) * 0.94) - 0.06);
        }
        
        turnMotor.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));

    }

    public double getError() {
        return driveMotor.getClosedLoopError();
    }

    public double getWantedSpeed() {
        return wantedSpeed;
    }

    public void stop() {
        wantedSpeed = 0;
        //driveMotor.set(ControlMode.Velocity, 0);
        driveMotor.set(ControlMode.PercentOutput, 0);
        turnMotor.set(0);
    }


    //EXPERIMENTAL METHODS:

    /**
     * Uses a PID to set drive velocity to 0
     */
    public void semiAutoStop() {
        driveMotor.set(ControlMode.PercentOutput, drivePID.calculate(getDriveVelocity(), 0));
        turnMotor.set(0);
    }

    public void fullStop() {
        driveMotor.set(ControlMode.Position, 0);
        turnMotor.set(0);
    }
    /**
     * Sets wheels to X formation
     */
    public void activeStop(int direction) {
        System.out.println("2\n2\n2\n2\n2\n2\n2\n2");
        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(0.785398 * direction));
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput, drivePID.calculate(getDriveVelocity(), 0));
        turnMotor.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
    }

}