package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.util.swerve.SwerveModuleConstants;

public final class SwerveConstants {
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.5); //TODO: This must be tuned to specific robot
    public static final double wheelBase = Units.inchesToMeters(26.75); //TODO: This must be tuned to specific robot
    public static final double wheelCircumference = Units.inchesToMeters(4.0) * Math.PI;

    /* Swerve Kinematics 
        * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = (6.75 / 1.0); // 5.14:1
    public static final double angleGearRatio = (150/7 / 1.0); // 12.8:1

    /* Motor Inverts */
    public static final boolean angleMotorInvert = true;
    public static final boolean driveMotorInvert = false;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.Clockwise_Positive;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 2;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 5;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
        * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.0;//0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = 45; // 43
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.2;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    /* Drive Motor Characterization Values 
        * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 5.0; //TODO: This must be tuned to specific robot
    public static final double maxWheelSpeed = 5200/driveGearRatio/60*wheelCircumference;//rpm/gear ratio*circumference/60rps
    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0;// Radians per second //TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { //TODO: This must be tuned to specific robot
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(
                40,
                4,
                4,
                45-8//TODO angle offset in degrees
                );
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { //TODO: This must be tuned to specific robot
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(
                10,
                1,
                1,
                0-8// TODO angle offset in degrees
                );
    }
    
    /* Back Left Module - Module 2 */
    public static final class Mod2 { //TODO: This must be tuned to specific robot
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(
                30,
                3,
                3,
                -119.5-8// TODO angle offset in degrees
                );
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { //TODO: This must be tuned to specific robot
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(
                20,
                2,
                2,
                56.7-8//TODO angle offset in degrees
                );
    }
        
}
