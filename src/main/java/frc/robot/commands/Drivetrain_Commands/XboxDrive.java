package frc.robot.commands.Drivetrain_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainFalcon;

import java.util.function.DoubleSupplier;

public class XboxDrive extends CommandBase {
    // Adding subsystem
    private DrivetrainFalcon m_drivetrain;

    // Adding DoubleSupplier of axis
    private DoubleSupplier joyY;
    private DoubleSupplier joyX;
    private DoubleSupplier joyZ;

    public XboxDrive(DrivetrainFalcon drivetrain, DoubleSupplier joyY, DoubleSupplier joyX) {
        this(drivetrain, joyY, joyX, () -> 1.0);
    }

    public XboxDrive(DrivetrainFalcon drivetrain, DoubleSupplier joyY, DoubleSupplier joyX, DoubleSupplier joyZ) {
        m_drivetrain = drivetrain;
        this.joyY = joyY;
        this.joyX = joyX;
        this.joyZ = joyZ;

        addRequirements(m_drivetrain);

    }
}
