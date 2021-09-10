// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autonomous.AutonomousGSC;
import frc.robot.commands.Autonomous.TrajectoryBase;
import frc.robot.commands.Drivetrain_Commands.AlignShooter;
import frc.robot.commands.Drivetrain_Commands.JoystickDrive;
import frc.robot.commands.Drivetrain_Commands.ShiftHigh;
import frc.robot.commands.Harm_Commands.IntakeGroup;
import frc.robot.commands.Shooter_Commands.ShootDemo;
import frc.robot.commands.Shooter_Commands.ShootVelocity;
import frc.robot.subsystems.DrivetrainFalcon;
import frc.robot.subsystems.Harm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shifter;
import frc.robot.util.PIDControllerDebug;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // User Input
  private static Joystick stick = new Joystick(0);
  
  private static JoystickButton button1 = new JoystickButton(stick, 1);
  private static JoystickButton button2 = new JoystickButton(stick, 2);
  private static JoystickButton button3 = new JoystickButton(stick, 3);
  private static JoystickButton button4 = new JoystickButton(stick, 4);
  private static JoystickButton button6 = new JoystickButton(stick, 6);  
  private static JoystickButton button10 = new JoystickButton(stick, 10); 

  // Subsystems
  private final Shooter shooter = new Shooter();
  private final DrivetrainFalcon drivetrain = new DrivetrainFalcon();
  private final Harm harm = new Harm();
  private final Shifter shifter = new Shifter();


  private final Compressor compressor = new Compressor();

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private enum Color {
    UNLOADED(0),
    RED(1),
    BLUE(2);

    private int value;
    private static Map<Integer, Color> map = new HashMap<>();

    private Color(int value) {
      this.value = value;
    }

    static {
      for (Color color : Color.values()) {
        map.put(color.value, color);
      }
    }

    public static Color valueOf(int color) {
      return (Color) map.get(color);
    }

    public int getValue() {
      return value;
    }
  }

  private static AtomicReference<Color> pathColor = new AtomicReference<>(Color.UNLOADED);

  private enum Letter {
    UNLOADED(0),
    A(1),
    B(2);

    private int value;
    private static Map<Integer, Letter> map = new HashMap<>();

    private Letter(int value) {
      this.value = value;
    }

    static {
      for (Letter letter : Letter.values()) {
        map.put(letter.value, letter);
      }
    }

    public static Letter valueOf(int letter) {
      return (Letter) map.get(letter);
    }

    public int getValue() {
      return value;
    }
  }

  private static AtomicReference<Letter> pathLetter = new AtomicReference<>(Letter.UNLOADED);

  private static enum ShooterMode {
    COMPETITION,
    DEMO
  }

  private static SendableChooser<ShooterMode> shooterMode = new SendableChooser<>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    // Configure the button bindings
    System.out.print("Testing Robot Container");
    // initializeTrajectory must come before configureButtonBindings
    configureButtonBindings();
    DriverStation.getInstance().silenceJoystickConnectionWarning(true);

    compressor.start();
    
    drivetrain.setDefaultCommand(new JoystickDrive(
      drivetrain,
      () -> -stick.getY() * (button3.get() ? -1.0 : 1.0),  // Because Negative Y is forward on the joysticks
      () -> stick.getX(),
      () -> (stick.getZ() - 1)/-2.0
    ));

    initializeAutonomousOptions();

    shooterMode.setDefaultOption("Competition Shooting", ShooterMode.COMPETITION);
    shooterMode.addOption("Demo Shooting", ShooterMode.DEMO);
    SmartDashboard.putData(shooterMode);
  }

  private void initializeAutonomousOptions() {
    
    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Bounce Piece", new SequentialCommandGroup(
      new TrajectoryBase(drivetrain, "/BOUNCE-1", false, true), // ... boolean isBackwards, boolean resetGyro
      new TrajectoryBase(drivetrain, "/BOUNCE-2", true, false),
      new TrajectoryBase(drivetrain, "/BOUNCE-3", false, false),
      new TrajectoryBase(drivetrain, "/BOUNCE-4", true, false)
    ));
    m_chooser.addOption("Bounce Path", new SequentialCommandGroup(
      new TrajectoryBase(drivetrain, "/BOUNCE-1", false, true), // ... boolean isBackwards, boolean resetGyro
      new TrajectoryBase(drivetrain, "/BOUNCE-2", true, false),
      new TrajectoryBase(drivetrain, "/BOUNCE-3", false, false),
      new TrajectoryBase(drivetrain, "/BOUNCE-4", true, false)
    ));
    m_chooser.addOption("Slalom Path",
      new TrajectoryBase(drivetrain, "/SLALOM")
    );

    System.out.println("Path Color: " + pathColor.get());
    m_chooser.addOption("Galactic Search Challenge",
      new ParallelCommandGroup(
        new AutonomousGSC(
        drivetrain,
        harm,
        () -> ((int) SmartDashboard.getNumber("isRed", 0) != 0), //&& pathLetter.get() != Letter.UNLOADED),
        () -> ((int) SmartDashboard.getNumber("isRed", 0) == 1),
        () -> (pathLetter.get() == Letter.A)
      )
    ));

    m_chooser.addOption("Galactic Red",
      new ParallelCommandGroup(
        new TrajectoryBase(drivetrain, "/RED-COMBINED", true, false).deadlineWith(
        new IntakeGroup(harm))
      )
    );

    // m_chooser.addOption("Galactic Search Challenge B", new AutonomousGSC_B(
    //   drivetrain,
    //   harm,
    //   () -> (pathColor.get() != Color.UNLOADED),
    //   () -> (pathColor.get() == Color.RED)
    // ));

    m_chooser.addOption("Forward Then Backward Path", new SequentialCommandGroup(
      new TrajectoryBase(drivetrain, "/FORWARD-DISTANCE", false, true), // ... boolean isBackwards, boolean resetGyro
      new TrajectoryBase(drivetrain, "/BACKWARD-DISTANCE", true, false)
    ));

    SmartDashboard.putData(m_chooser);
    SmartDashboard.putData(turnDirection_chooser);

    // NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // NetworkTable table = inst.getTable("SmartDashboard");

    // table.addEntryListener("isRed",
    //   (local_table, key, entry, value, flags) -> {
    //     pathColor.set(Color.valueOf((int) value.getValue()));
    //   },
    //   EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
    // );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    SmartDashboard.putData(new InstantCommand(
      () -> drivetrain.resetEncoders(),
      drivetrain
    ) {
      @Override
      public void initialize() {
        super.initialize();
        setName("Reset Encoders");
      }

      @Override
      public boolean runsWhenDisabled() {
        return true;
      }
    });

    // Shooter: Determines shooting mode based on SmartDashboard chooser
    button2.whileHeld(new ConditionalCommand(
      new ShootVelocity(shooter, compressor, harm, () -> !button6.get()),
      new ShootDemo(shooter, compressor, harm),
      () -> (shooterMode.getSelected() == ShooterMode.COMPETITION)
    ));

    button4.whileHeld(new ShiftHigh(shifter));
    // button6.whileHeld(new RaiseShooterHood(harm));  // .whenReleased(new LowerShooterHood(harm));
    
    button1.whileHeld(new IntakeGroup(harm));

    // PIDController pidcontroller = new PIDControllerDebug(0.0006, 0.0005, 0.0);
    PIDController pidcontroller = new PIDControllerDebug(0.002, 0.001, 0.0);
    pidcontroller.setIntegratorRange(-0.15, 0.15);

    if (Robot.isSimulation()) {
      SmartDashboard.putNumber("TargetX", 0);
    }

    button10.whileHeld(new AlignShooter(pidcontroller, 
    () -> {
        double x = SmartDashboard.getNumber("TargetX", -9999);
        // System.out.println(String.format("Info: x %f", x));
        if (x == -9999) {
          return 0;
        } 
        return x;
      },
    0.0,
    (output) -> {
        output = MathUtil.clamp(output, -.5, .5);
        
        drivetrain.arcadeDrive(0, -output, false);
        // System.out.println(String.format("Info: output %f", output));
      },
    drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public void stopAllSubsystems(){
    drivetrain.stop();
  }

  public void periodic() {
    drivetrain.periodic();
  }
}