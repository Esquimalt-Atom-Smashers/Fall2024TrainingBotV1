package frc.robot.commands;

import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class FollowTargets extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private SwerveSubsystem swerveSubsystem;
    private LimelightSubsystem m_LimelightSubsystem;
    private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public FollowTargets(SwerveSubsystem subsystem_drive, LimelightSubsystem subsystem_limelight) {
       swerveSubsystem = subsystem_drive;
       m_LimelightSubsystem= subsystem_limelight;
      // // Use addRequirements() here to declare subsystem dependencies.
       addRequirements(subsystem_drive);
      // 
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      double maxFollowingSpeed =0.4;
      double followingDistanceM =1.5;
      // Use addRequirements() here to declare subsystem dependencies.
      double targetPosition = m_LimelightSubsystem.getTargets()[0];
      double targetErrorZ = Math.min(maxFollowingSpeed,Math.max(-maxFollowingSpeed, m_LimelightSubsystem.getTargetPose()[2]-followingDistanceM));
      double rot = MathUtil.applyDeadband(Math.min(Math.max((targetPosition/Math.abs(targetPosition))*Math.sqrt(Math.abs(targetPosition/20)),-SwerveConstants.MAX_ANG_VEL_RAD_S),SwerveConstants.MAX_ANG_VEL_RAD_S),0.5);
      double speedX = xspeedLimiter.calculate(targetErrorZ);
      swerveSubsystem.driveMPS(new Translation2d(speedX,0.0),rot,false);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }
  
