package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PigeonGyro extends SubsystemBase{
    Pigeon2 gyro = new Pigeon2(0);

    public double getPitch(){
        double pitch = gyro.getPitch().getValueAsDouble();
        return pitch;
    }
    public double getYaw(){
        double yaw = gyro.getYaw().getValueAsDouble();
        return yaw;
    }
    public double getRoll(){
        double roll = gyro.getRoll().getValueAsDouble();
        return roll;
    }
    public Rotation2d getYawRotation2d(){
        Rotation2d yaw = Rotation2d.fromDegrees(getYaw());
        return yaw;
    }
    public void reset(){
        //gyro.reset();
        gyro.setYaw(0);
    }
}
