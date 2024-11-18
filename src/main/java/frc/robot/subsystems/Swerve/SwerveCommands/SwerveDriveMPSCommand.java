package frc.robot.subsystems.Swerve.SwerveCommands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class SwerveDriveMPSCommand extends Command {    
    private SwerveSubsystem s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier fieldCentricSup;

    public SwerveDriveMPSCommand(SwerveSubsystem s_Swerve, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotationRadPerS, BooleanSupplier fieldCentric) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = forward;
        this.strafeSup = strafe;
        this.rotationSup = rotationRadPerS;
        this.fieldCentricSup = fieldCentric;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = translationSup.getAsDouble();
        double strafeVal = strafeSup.getAsDouble();
        double rotationVal = rotationSup.getAsDouble();

        /* Drive */
        s_Swerve.driveMPS(
            new Translation2d(translationVal, strafeVal), 
            rotationVal * Constants.Teleop.maxAngSpeedRadPerS, 
            fieldCentricSup.getAsBoolean()
        );
    }
}