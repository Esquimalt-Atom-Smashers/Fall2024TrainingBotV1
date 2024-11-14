package frc.util.swerve;

//was called CTREConfigs

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
//import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;

public class FalconSwerveConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;

    public FalconSwerveConfigs() {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve Angle Motor Configurations */
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();
        angleSupplyLimit.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        angleSupplyLimit.SupplyCurrentLimit = Constants.Swerve.angleContinuousCurrentLimit;
        //time threshold and current threshold do not seem to do what they are supposed to do when using falcons
        //angleSupplyLimit.SupplyCurrentThreshold = Constants.Swerve.anglePeakCurrentLimit;
        angleSupplyLimit.SupplyTimeThreshold = 0.0; //hardcoded to prevent activation

        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;
        //swerveAngleFXConfig.Slot0.kS = Constants.Swerve.angleKS;
        //swerveAngleFXConfig.Slot0.kV = Constants.Swerve.angleKV;
        swerveAngleFXConfig.CurrentLimits = angleSupplyLimit;

        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();
        driveSupplyLimit.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        driveSupplyLimit.SupplyCurrentLimit = Constants.Swerve.driveContinuousCurrentLimit;
        //time threshold and current threshold do not seem to do what they are supposed to do when using falcons
        //driveSupplyLimit.SupplyCurrentThreshold = Constants.Swerve.drivePeakCurrentLimit;
        driveSupplyLimit.SupplyTimeThreshold = 0.0; //hardcoded to prevent activation

        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.Slot0.kS = Constants.Swerve.driveKS;
        swerveDriveFXConfig.Slot0.kV = Constants.Swerve.driveKV;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.CurrentLimits = driveSupplyLimit;

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.canCoderInvert;

        /* does not set the magnet offset currently due to needing to set magnet offset to trim */
    }
}
