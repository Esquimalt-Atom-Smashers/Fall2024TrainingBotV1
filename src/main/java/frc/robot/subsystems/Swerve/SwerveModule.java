package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.util.math.SwerveConversions;
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
//import com.ctre.phoenix6.controls.VoltageOut;
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
    //private VoltageOut voltageRequestDrive = new VoltageOut(0);

    public SwerveModuleState getState(){
        return new SwerveModuleState(getSpeed(),getAngle()); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            SwerveConversions.falconToMeters(mDriveMotor.getPosition().refresh().getValue()),
            //TODO Add a correction factor for coupling between the azimuth and drive gears. 
            // every time azimuth spins, it affects the real position of the drive wheel, and 
            //therefore affects odometry.
            getAngle()
        );
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().refresh().getValue());
    }

    public void resetToAbsolute(){//TODO find out if we need to do this since we are using the canCoder as a Remote encoder
        double absolutePosition = SwerveConversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), SwerveConstants.angleGearRatio);
        mAngleMotor.setPosition(absolutePosition);
    }
    
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
        desiredState = optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            //double voltageOutput = 12*desiredState.speedMetersPerSecond / SwerveConstants.maxWheelSpeed;
            //mDriveMotor.setControl(voltageRequestDrive.withOutput(voltageOutput));
        }
        else {
            double velocityMotorRPS = SwerveConversions.MPSToKraken(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(velocityVoltageRequestDrive.withVelocity(velocityMotorRPS)); 
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
        CurrentLimitsConfigs angleCurrentLimits = new CurrentLimitsConfigs();
        angleCurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.angleEnableCurrentLimits;
        angleCurrentLimits.SupplyCurrentLimit = SwerveConstants.angleSupplyCurrentLimit;
        angleCurrentLimits.SupplyTimeThreshold = 0.0; //hardcoded to prevent activation
        angleCurrentLimits.StatorCurrentLimitEnable =SwerveConstants.angleEnableCurrentLimits;
        angleCurrentLimits.StatorCurrentLimit=SwerveConstants.angleStatorCurrentLimit;
        
        TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
        swerveAngleFXConfig.Slot0.kP = SwerveConstants.angleKP;
        swerveAngleFXConfig.Slot0.kI = SwerveConstants.angleKI;
        swerveAngleFXConfig.Slot0.kD = SwerveConstants.angleKD;
        swerveAngleFXConfig.CurrentLimits = angleCurrentLimits;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;;
        
        TalonFXConfigurator configurator = mAngleMotor.getConfigurator();
        configurator.apply(swerveAngleFXConfig);

        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

        outputConfigs.Inverted = SwerveConstants.angleMotorInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        outputConfigs.NeutralMode = SwerveConstants.angleNeutralMode;

        configurator.apply(outputConfigs);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;//fused cancoder is pro only
        //feedbackConfigs.RotorToSensorRatio = SwerveConstants.angleGearRatio;
        configurator.apply(feedbackConfigs);
    }

    private void configDriveMotor(){  
        
        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveCurrentLimits = new CurrentLimitsConfigs();
        driveCurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.driveEnableCurrentLimits;
        driveCurrentLimits.SupplyCurrentLimit = SwerveConstants.driveSupplyCurrentLimit;
        driveCurrentLimits.SupplyTimeThreshold = 0.0; //hardcoded to prevent activation
        driveCurrentLimits.StatorCurrentLimitEnable = SwerveConstants.driveEnableCurrentLimits;
        driveCurrentLimits.StatorCurrentLimit=SwerveConstants.driveStatorCurrentLimit;
        TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();

        swerveDriveFXConfig.Slot0.kP = SwerveConstants.driveKP;
        swerveDriveFXConfig.Slot0.kI = SwerveConstants.driveKI;
        swerveDriveFXConfig.Slot0.kD = SwerveConstants.driveKD;
        swerveDriveFXConfig.Slot0.kS = SwerveConstants.driveKS;
        swerveDriveFXConfig.Slot0.kV = SwerveConstants.driveKV;
        swerveDriveFXConfig.Slot0.kA = SwerveConstants.driveKA;
        swerveDriveFXConfig.CurrentLimits = driveCurrentLimits;

        TalonFXConfigurator configurator = mDriveMotor.getConfigurator();
        configurator.apply(swerveDriveFXConfig);

        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

        outputConfigs.NeutralMode = SwerveConstants.driveNeutralMode;
        outputConfigs.Inverted = SwerveConstants.driveMotorInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        
        configurator.apply(outputConfigs);
    }



    /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to include placing
   * in appropriate scope for CTRE onboard control.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  private static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    //TODO check if this is why some wheels appeared to be oriented the opposite way during testing
    //TODO add boolean to make motors go to same orientation on startup may not work with static method
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90){
        targetSpeed = -targetSpeed;
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }        
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
      double lowerBound;
      double upperBound;
      double lowerOffset = scopeReference % 360;
      if (lowerOffset >= 0) {
          lowerBound = scopeReference - lowerOffset;
          upperBound = scopeReference + (360 - lowerOffset);
      } else {
          upperBound = scopeReference - lowerOffset;
          lowerBound = scopeReference - (360 + lowerOffset);
      }
      while (newAngle < lowerBound) {
          newAngle += 360;
      }
      while (newAngle > upperBound) {
          newAngle -= 360;
      }
      if (newAngle - scopeReference > 180) {
          newAngle -= 360;
      } else if (newAngle - scopeReference < -180) {
          newAngle += 360;
      }
      return newAngle;
  }
}