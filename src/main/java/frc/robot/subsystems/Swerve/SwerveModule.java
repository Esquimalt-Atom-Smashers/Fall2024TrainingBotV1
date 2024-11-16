package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.util.math.SwerveConversions;
import frc.util.swerve.CTREModuleState;
import frc.util.swerve.SwerveModuleConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private PositionVoltage positionVoltageRequestAngle = new PositionVoltage(0).withSlot(0);
    private VelocityVoltage velocityVoltageRequestDrive = new VelocityVoltage(0).withSlot(0);
    private VoltageOut voltageRequestDrive = new VoltageOut(0);
    
    protected SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    protected void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double voltageOutput = 12*desiredState.speedMetersPerSecond / SwerveConstants.maxWheelSpeed;
            mDriveMotor.setControl(voltageRequestDrive.withOutput(voltageOutput));
        }
        else {
            double velocity = SwerveConversions.MPSToFalcon(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(velocityVoltageRequestDrive.withVelocity(velocity)); 
            //removed the feedforward part because I don't think it was being used right.    
        }
    }

    //TODO go over this to make sure the angle makes sense
    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxWheelSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        double angleD= angle.getDegrees();
        mAngleMotor.setControl(positionVoltageRequestAngle.withPosition(angleD/360));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromRotations(BaseStatusSignal.getLatencyCompensatedValue(mAngleMotor.getPosition(), mAngleMotor.getVelocity()));
    }
    private double getSpeed() {
        return SwerveConversions.falconToMPS(mDriveMotor.getVelocity().refresh().getValue());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().refresh().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = SwerveConversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), SwerveConstants.angleGearRatio);
        mAngleMotor.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){   
        CANcoderConfiguration swerveCanCoderConfig= new CANcoderConfiguration();
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = SwerveConstants.canCoderInvert;

        angleEncoder.getConfigurator().apply(swerveCanCoderConfig);
        MagnetSensorConfigs myMagnetSensorConfigs = new MagnetSensorConfigs();
        angleEncoder.getConfigurator().refresh(myMagnetSensorConfigs);
        myMagnetSensorConfigs.MagnetOffset = -angleOffset.getDegrees() / 360;
        angleEncoder.getConfigurator().apply(myMagnetSensorConfigs);
    }

    private void  configAngleMotor() {
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();
        angleSupplyLimit.SupplyCurrentLimitEnable = SwerveConstants.angleEnableCurrentLimit;
        angleSupplyLimit.SupplyCurrentLimit = SwerveConstants.angleContinuousCurrentLimit;
        angleSupplyLimit.SupplyTimeThreshold = 0.0; //hardcoded to prevent activation
        
        TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
        swerveAngleFXConfig.Slot0.kP = SwerveConstants.angleKP;
        swerveAngleFXConfig.Slot0.kI = SwerveConstants.angleKI;
        swerveAngleFXConfig.Slot0.kD = SwerveConstants.angleKD;
        //swerveAngleFXConfig.Slot0.kS = angleKS;
        //swerveAngleFXConfig.Slot0.kV = angleKV;
        swerveAngleFXConfig.CurrentLimits = angleSupplyLimit;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;;
        
        TalonFXConfigurator configurator = mAngleMotor.getConfigurator();
        configurator.apply(swerveAngleFXConfig);

        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

        outputConfigs.Inverted = SwerveConstants.angleMotorInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        outputConfigs.NeutralMode = SwerveConstants.angleNeutralMode;

        configurator.apply(outputConfigs);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        feedbackConfigs.RotorToSensorRatio = SwerveConstants.angleGearRatio;
        configurator.apply(feedbackConfigs);
    }

    private void configDriveMotor(){  
        
        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();
        driveSupplyLimit.SupplyCurrentLimitEnable = SwerveConstants.driveEnableCurrentLimit;
        driveSupplyLimit.SupplyCurrentLimit = SwerveConstants.driveContinuousCurrentLimit;
        driveSupplyLimit.SupplyTimeThreshold = 0.0; //hardcoded to prevent activation
        TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();

        swerveDriveFXConfig.Slot0.kP = SwerveConstants.driveKP;
        swerveDriveFXConfig.Slot0.kI = SwerveConstants.driveKI;
        swerveDriveFXConfig.Slot0.kD = SwerveConstants.driveKD;
        swerveDriveFXConfig.Slot0.kS = SwerveConstants.driveKS;
        swerveDriveFXConfig.Slot0.kV = SwerveConstants.driveKV;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;
        swerveDriveFXConfig.CurrentLimits = driveSupplyLimit;

        TalonFXConfigurator configurator = mDriveMotor.getConfigurator();
        configurator.apply(swerveDriveFXConfig);

        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

        outputConfigs.NeutralMode = SwerveConstants.driveNeutralMode;
        outputConfigs.Inverted = SwerveConstants.driveMotorInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        
        configurator.apply(outputConfigs);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getSpeed(),getAngle()); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            SwerveConversions.falconToMeters(mDriveMotor.getPosition().refresh().getValue()), 
            getAngle()
        );
    }
}