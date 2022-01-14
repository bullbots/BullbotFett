package frc.robot.subsystems;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.SafeSparkMax;
import frc.robot.util.SafeTalonFX;

public class NEO_Shooter extends SubsystemBase {
     // Inistalizing NEO Motors
     private SafeSparkMax bottom_shooter;
     private SafeSparkMax top_shooter;

     //Inistalizing FALCON Motors
     private SafeTalonFX top_fx_shooter;
     private SafeTalonFX bottom_fx_shooer;

     // Inistalizing Servo
     public final Servo ballReleaseServo = new Servo(Constants.RELEASE_SERVO_PORT);

     //Adding Vlaues to Network table
     private NetworkTableEntry topVelocity;
     private NetworkTableEntry bottomVelocity;

     //Intaking Values to determine velocity
     List<Integer> velocityRange = IntStream.rangeClosed(-100, 100).boxed().collect(Collectors.toList());
     private int curTopVelIndex = 0;
     private int curBottomVelIndex = velocityRange.size() / 2;

     private enum MotorPlacement {
         BOTTOM, TOP
     }

     public NEO_Shooter() {
         top_fx_shooter = new SafeTalonFX(Constants.TOP_SHOOTER_PORT, true);
         bottom_shooter = new SafeSparkMax(Constants.BOTTOM_SHOOTER_PORT, MotorType.kBrushless);

         configurePID();

         top_fx_shooter.setNeutralMode(NeutralMode.Coast);
         bottom_shooter.setIdleMode(IdleMode.kCoast);

         setDefaultCommand(new RunCommand(() -> ballReleaseServo.set(1), this));

         var inst = NetworkTableInstance.getDefault();
     }

     public void configurePID() {
         // Configuring the FX top motors PID values
         top_fx_shooter.config_kF(0, Constants.SHOOTER_FF);
         top_fx_shooter.config_kP(0, Constants.SHOOTER_P);
         top_fx_shooter.config_kI(0, Constants.SHOOTER_I);
         top_fx_shooter.config_kP(0, Constants.SHOOTER_D);

         // Setting PID system to default
         bottom_shooter.restoreFactoryDefaults();

         // COnfiguring the NEO bottom motors PID values
         bottom_shooter.getPIDController().setFF(Constants.SHOOTER_FF);
         bottom_shooter.getPIDController().setP(Constants.SHOOTER_P);
         bottom_shooter.getPIDController().setI(Constants.SHOOTER_I);
         bottom_shooter.getPIDController().setP(Constants.SHOOTER_D);
     }

     // Stops the motors
     public void stop() {
         top_fx_shooter.stopMotor();
         bottom_shooter.stopMotor();
     }

     /**
      * THis gets the velocity of the motors
      * @param motorPlacement This is where the motor will be placed at the top or bottom slots
      * @return curVal This returns the current velocity of the shooter motors
      */
     public double getVelocity(MotorPlacement motorPlacement) {
         double curVal = 0;
         if (Robot.isReal()) {
             switch (motorPlacement) {
                case TOP:
                    curVal = top_fx_shooter.getSelectedSensorVelocity();
                    break;
                case BOTTOM:
                    curVal = bottom_shooter.get();
             }
         } else {
             int curIndex = 0;

             switch (motorPlacement) {
                case TOP:
                    curIndex = curTopVelIndex;
                    break;
                case BOTTOM:
                    curIndex = curBottomVelIndex;
            }

            curVal = velocityRange.get(curIndex);

            if(curIndex >= velocityRange.size() - 1) {
                switch (motorPlacement) {
                    case TOP:
                        curTopVelIndex = 0;
                        break;
                    case BOTTOM:
                        curBottomVelIndex = 0;
                        break;
                }
            } else {
                switch (motorPlacement) {
                    case TOP: 
                        curTopVelIndex++;
                        break;
                    case BOTTOM: 
                        curBottomVelIndex++;
                        break;
                }
            }
        }
        return curVal;
    }

    public double[] getVelocities() {
        double top_vel = top_fx_shooter.getSelectedSensorVelocity();
        double bottom_vel = bottom_shooter.get();

        return new double[] {top_vel, bottom_vel};
    }

    /**
     * This sets the velocity of the top and bottom shooter motors
     * @param top_vel This is the velocity of the top motor
     * @param bottom_vel This is the velocity of the bottom motor
     */
    public void set(double top_vel, double bottom_vel) {
        top_fx_shooter.set(top_vel);
        bottom_shooter.set(bottom_vel);
        System.out.println(String.format("Top Velocity %.02f Bottom Velocity %.02f", top_vel, bottom_vel));
    }

    public void periodic() {

    }
}