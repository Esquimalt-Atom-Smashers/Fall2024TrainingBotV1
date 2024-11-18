package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;

public final class SwerveConstants {
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(20.6); // This must be tuned to specific robot
    public static final double wheelBase = Units.inchesToMeters(25.5); // This must be tuned to specific robot
    public static final double wheelCircumference = Units.inchesToMeters(4.0) * Math.PI;
    public static final int maxDriveMotorRPM =6000;//6000 for kraken, 5200 for falcon

    /* Module Gear Ratios */
    public static final double driveGearRatio = (6.75 / 1.0); // 5.14:1
    public static final double angleGearRatio = (150/7 / 1.0); // 12.8:1 //TODO double check this value, 21 seems wrong

    /* Motor Inverts */
    public static final boolean angleMotorInvert = true;
    public static final boolean driveMotorInvert = false;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.Clockwise_Positive;

    /* Swerve Current Limiting */
    public static final int angleSupplyCurrentLimit = 20;
    public static final int angleStatorCurrentLimit = 8;
    public static final boolean angleEnableCurrentLimits = true;

    public static final int driveSupplyCurrentLimit = 40;
    public static final int driveStatorCurrentLimit = 20;
    public static final boolean driveEnableCurrentLimits = true;

    /* Angle Motor PID Values */
    public static final double angleKP = 45; // 43
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.2;
    public static final double driveKI = 0.0;//Don't use, KI value spools up too much
    public static final double driveKD = 0.01;

    /* Drive Motor Characterization Values 
        * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.4); //TODO: This must be tuned to specific robot
    public static final double driveKV = (12.7/maxDriveMotorRPM*60);//0.12 in all docs units are Volts per revolution per second
    public static final double driveKA = 0;//(0.27 / 12); //TODO: This is final robot weight dependant 

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxWheelSpeed = maxDriveMotorRPM/driveGearRatio/60*wheelCircumference;//rpm/gear ratio*circumference/60rps
    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0;// Radians per second //TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;//TODO: check if the wheel can turn by hand in angle
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class frontLeftModule { //TODO: This must be tuned to specific robot
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(
                40,
                4,
                4,
                45-8//TODO angle offset in degrees
                );
    }

    /* Front Right Module - Module 1 */
    public static final class frontRightModule { //TODO: This must be tuned to specific robot
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(
                10,
                1,
                1,
                0-8// TODO angle offset in degrees
                );
    }
    
    /* Back Left Module - Module 2 */
    public static final class rearLeftModule { //TODO: This must be tuned to specific robot
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(
                30,
                3,
                3,
                -119.5-8// TODO angle offset in degrees
                );
    }

    /* Back Right Module - Module 3 */
    public static final class rearRightModule { //TODO: This must be tuned to specific robot
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(
                20,
                2,
                2,
                56.7-8//TODO angle offset in degrees
                );
    }
        
}
