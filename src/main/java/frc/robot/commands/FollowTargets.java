package frc.robot.commands;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class FollowTargets extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private SwerveSubsystem swerveSubsystem;
    private LimelightSubsystem m_LimelightSubsystem;
  
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
      
      // Use addRequirements() here to declare subsystem dependencies.
      double targetPosition = m_LimelightSubsystem.getTargets()[0];
      //double rot = Math.min(Math.max((targetPosition/5)*(Math.abs(targetPosition)/5),-0.8),0.6);//limit speed 
      double rot = Math.min(Math.max((targetPosition/Math.abs(targetPosition))*Math.sqrt(Math.abs(targetPosition/20)),-0.8),0.8);
      swerveSubsystem.driveMPS(new Translation2d(0.0,0.0),rot,false);
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
  
