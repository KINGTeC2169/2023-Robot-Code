package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.ModuleConstants.*;

public class SwerveModule {
    private TalonFX driveMotor;
    private TalonFX turnMotor;
    private CANCoder absoluteEncoder;


    private PIDController drivePID;
    private PIDController turningPID;
    private CANCoderConfiguration config = new CANCoderConfiguration();

    public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorReversed, 
                boolean turnMotorReversed, int canCoderID, double absoluteOffset, 
                boolean isCancoderReversed) {

        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new TalonFX(turnMotorID);
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

        //driveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        /* 
        driveEncoder.setPositionConversionFactor(driveEncoderToMeter);
        driveEncoder.setVelocityConversionFactor(driveEncoderRPMToMeterPerSec);
        turnEncoder.setPositionConversionFactor(turnEncoderToRadian);
        turnEncoder.setVelocityConversionFactor(turnEncoderRPMToRadPerSec);
        */

        //Creating and configuring PID controllers
        turningPID = new PIDController(PTurn, 0, 0);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);

        //driveMotor.config_kP(0, .25);
        drivePID = new PIDController(PDrive, 0, 0);

        resetEncoders();

    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition() * driveEncoderToMeter;
    }

    public double getTurnPosition() {
        return turnMotor.getSelectedSensorPosition() * turnEncoderToRadian;
    }

    
    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() * driveEncoderRPMToMeterPerSec;
    }

    public double getTurnVelocity() {
        return turnMotor.getSelectedSensorVelocity() * turnEncoderRPMToRadPerSec;
        //>return turnMotor.getSelectedSensorVelocity();
    }

    public double getAbsoluteTurnPosition() {
        return absoluteEncoder.getAbsolutePosition();
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turnMotor.setSelectedSensorPosition(getAbsoluteTurnPosition());
        System.out.println("RESETTING ENCODERS \nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS");
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            //TODO: this causes an aggresive stop, need to test with more weight on bot
            //semiAutoStop();
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / maxSpeed);
        turnMotor.set(ControlMode.PercentOutput, turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(ControlMode.Velocity, 0);
        turnMotor.set(ControlMode.Velocity, 0);
    }


    //EXPERIMENTAL METHODS:

    /**
     * Uses a PID to set drive velocity to 0
     */
    public void semiAutoStop() {
        driveMotor.set(ControlMode.PercentOutput, drivePID.calculate(getDriveVelocity(), 0));
        turnMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Sets wheels to X formation
     */
    public void activeStop(int direction) {
        System.out.println("2\n2\n2\n2\n2\n2\n2\n2");
        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(0.785398 * direction));
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput, drivePID.calculate(getDriveVelocity(), 0));
        turnMotor.set(ControlMode.PercentOutput, turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
    }

}
