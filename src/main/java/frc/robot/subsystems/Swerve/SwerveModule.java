package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.Robot;
import frc.util.math.SwerveConversions;
import frc.util.swerve.CTREModuleState;
import frc.util.swerve.SwerveModuleConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.hardware.CANcoder;

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
    
    SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(Swerve.driveKS, Swerve.driveKV, Swerve.driveKA);

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
            double voltageOutput = 12*desiredState.speedMetersPerSecond / Swerve.maxSpeed;
            mDriveMotor.setControl(voltageRequestDrive.withOutput(voltageOutput));
        }
        else {
            double velocity = SwerveConversions.MPSToFalcon(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(velocityVoltageRequestDrive.withVelocity(velocity)
                .withFeedForward(driveFeedforward.calculate(desiredState.speedMetersPerSecond)));    
        }
    }

    //TODO go over this to make sure the angle makes sense
    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
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
        double absolutePosition = SwerveConversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Swerve.angleGearRatio);
        mAngleMotor.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
        MagnetSensorConfigs myMagnetSensorConfigs = new MagnetSensorConfigs();
        angleEncoder.getConfigurator().refresh(myMagnetSensorConfigs);
        myMagnetSensorConfigs.MagnetOffset = -angleOffset.getDegrees() / 360;
        angleEncoder.getConfigurator().apply(myMagnetSensorConfigs);
    }

    private void  configAngleMotor() {
        TalonFXConfigurator configurator = mAngleMotor.getConfigurator();
        configurator.apply(Robot.ctreConfigs.swerveAngleFXConfig);

        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

        outputConfigs.Inverted = Constants.Swerve.angleMotorInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        outputConfigs.NeutralMode = Constants.Swerve.angleNeutralMode;

        configurator.apply(outputConfigs);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        feedbackConfigs.RotorToSensorRatio = Constants.Swerve.angleGearRatio;
        configurator.apply(feedbackConfigs);
    }

    private void configDriveMotor(){        
        TalonFXConfigurator configurator = mDriveMotor.getConfigurator();
        configurator.apply(Robot.ctreConfigs.swerveDriveFXConfig);

        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

        outputConfigs.NeutralMode = Constants.Swerve.driveNeutralMode;
        outputConfigs.Inverted = Constants.Swerve.driveMotorInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        
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