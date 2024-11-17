package frc.robot;

public final class Constants {
    
    public static final class Teleop {
        public static final double stickDeadband = 0.12;
        public static final double kMaxSpeedMetersPerSecond = 2.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;


    }

    public static final class Auto { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    }
}
