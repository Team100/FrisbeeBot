package frc.robot;

import org.team100.lib.sensors.AHRS;
import org.team100.lib.sensors.LSM6DSOX_I2C;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import java.io.Console;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.control.FieldRelativeLaundryStick;
import frc.robot.control.RobotRelativeLaundryStick;
import frc.robot.subsystems.DirectLaundryDrive;
import frc.robot.subsystems.FieldRelativeLaundryDrive;
import frc.robot.subsystems.LaundryArm;
import frc.robot.subsystems.LaundryDrive;
import frc.robot.subsystems.StabilizedLaundryDrive;

/**
 * An example of functional design: subsystems "pull" commands from suppliers.
 * This is the reverse of the common imperative style, where commands are
 * "pushed" to consumers.
 */
public class Robot extends TimedRobot {
    /** To try field-relative control. */
    private static final boolean kFieldRelative = false;
    /** To try yaw stabilization, make this true. */
    private static final boolean kStabilize = false;

    private final LaundryDrive m_drive;
    private final LaundryArm m_arm;
    JoystickButton button;
    JoystickButton wheelButton;

    VictorSP wheelDrive;
    VictorSP wheelDrive2;

    // Solenoid exampleSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    // DoubleSolenoid exampleDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    
    Solenoid exampleSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    Solenoid exampleSolenoidPCM1 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);


    // private final PWM pwm = new PWM(5);

    public Robot() {
        DataLogManager.start();
        
        Talon leftMotor = new Talon(0);
        Talon rightMotor = new Talon(1);

         wheelDrive = new VictorSP(2);
         wheelDrive2 = new VictorSP(3);

        

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

            Joystick joystick = new Joystick(0);

            button = new JoystickButton(joystick, 1);
            wheelButton = new JoystickButton(joystick, 5);
            RobotRelativeLaundryStick stick = new RobotRelativeLaundryStick(joystick);
            m_drive = new DirectLaundryDrive(
                    stick::xSpeed1_1,
                    stick::zSpeed1_1,
                    drive);

            CANSparkMax armMotor = new CANSparkMax(2, MotorType.kBrushless);
            ProfiledPIDController armController = new ProfiledPIDController(
                    1.5, // P
                    0, // I
                    0, // D
                    new Constraints(
                            1, // max velocity (infinite)
                            1)); // max accel (infinite)
            m_arm = new LaundryArm(stick::dump, armController, armMotor);
        
    }
    @Override
    public void autonomousInit() {
        m_arm.autonomousInit();
        m_drive.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {
        if (m_arm.placeFinished()) {
            m_arm.autonomousPeriodic();
            m_drive.autonomousPeriodic();
            return;
        }
        m_arm.autonomousPeriodic();
    }
    @Override
    public void teleopInit() {
        m_drive.enable();
        m_arm.enable();
    }

    @Override
    public void teleopExit() {
        m_drive.disable();
        m_arm.disable();
    }

    @Override
    public void teleopPeriodic() {
        m_drive.teleopPeriodic();
        m_arm.periodic();

        if(wheelButton.getAsBoolean()){
            System.out.println("AHHHHHHHHHH");

            wheelDrive.set(1);
            wheelDrive2.set(-1);

        } else {
            System.out.println("BOOOO");

            wheelDrive.set(0);
            wheelDrive2.set(0);
        }


        // exampleSolenoidPCM.set(true);

        if(button.getAsBoolean()){
            exampleSolenoidPCM1.set(true);
            exampleSolenoidPCM.set(false);
        } else{
            exampleSolenoidPCM1.set(false);
            exampleSolenoidPCM.set(true);
        }
    }

    
}
