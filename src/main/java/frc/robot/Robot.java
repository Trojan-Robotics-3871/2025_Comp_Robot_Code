// FRC Imports - DO NOT REMOVE!
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;

// Venom Imports -
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;

// REV Imports -
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Robot extends TimedRobot {

  private static final String AutonomousA = "Autonomous Middle";
  private static final String AutonomousB = "Autonomous Left";
  private static final String AutonomousC = "Autonomous Right";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Tank drive wheels -
  CANVenom RightFront = new CANVenom(1); // Serial 6701
  CANVenom LeftFront = new CANVenom(2); // Serial 6739
  CANVenom RightRear = new CANVenom(3); // Serial 6682
  CANVenom LeftRear = new CANVenom(4); // Serial 6725

  // Spark Motors -
  SparkMax CoralMotor = new SparkMax(5, MotorType.kBrushless);
  SparkMax Winch = new SparkMax(6, MotorType.kBrushed);

  // Create pneumatics controllers -
  private static final int PH_CAN_ID = 11;
  PneumaticHub m_ph = new PneumaticHub(PH_CAN_ID);
  public static int forwardChannel1 = 0;
  public static int reverseChannel1 = 1;
  DoubleSolenoid m_doubleSolenoid = m_ph.makeDoubleSolenoid(forwardChannel1, reverseChannel1);

  // Xbox Controller Configuration -
  XboxController Controller = new XboxController(0);

  // Create battery Configuration -
  private PowerDistribution pdp = new PowerDistribution();
  double BatteryVoltage = pdp.getVoltage();

  // Helper function to calculate joystick angle
  public double getJoystickAngle(double x, double y) {
    double radians = Math.atan2(y, x);
    double degrees = Math.toDegrees(radians);
    // Normalize the angle to be between 0 and 360 degrees
    return (degrees + 360) % 360;
  }

  // Helper function to calculate joystick magnitude
  public double getJoystickMagnitude(double x, double y) {
    return Math.sqrt(x * x + y * y);
  }

  // Create drive motors, configured for turning based on joystick angle
  public void setDriveMotorsFromJoystick(double x, double y) {
    double K = getJoystickMagnitude(x, y); // Magnitude of joystick input
    double A = getJoystickAngle(x, y); // Angle of joystick input

    double leftSpeed = 0;
    double rightSpeed = 0;

    // Adding a small threshold to avoid deadzone issues between 350° and 0°
    double angleThreshold = 15; // This is the deadzone threshold (adjust as necessary)
    if (A >= 360 - angleThreshold || A <= angleThreshold) {
      A = 0; // Set to zero if within a small range near 360 or 0 degrees
    }

    // Check which angle range A falls into, and set left and right motor speeds
    if (90 <= (A + 45) && (A + 45) < 180) { // Forward
      leftSpeed = K;
      rightSpeed = K;
    } else if (180 <= (A + 45) && (A + 45) < 270) { // Left
      leftSpeed = K;
      rightSpeed = -K;
    } else if (270 <= (A + 45) && (A + 45) < 360) { // Backwards
      leftSpeed = -K;
      rightSpeed = -K;
    } else if (0 <= (A + 45) && (A + 45) < 90) { // Right
      leftSpeed = -K;
      rightSpeed = K;
    }

    // Configuration for modifying drive wheels speed
    double maxSpeed = 0.75;
    leftSpeed = Math.max(-maxSpeed, Math.min(leftSpeed, maxSpeed));
    rightSpeed = Math.max(-maxSpeed, Math.min(rightSpeed, maxSpeed));

    // Set motor speeds and invert direction for follower motors
    LeftFront.set(-leftSpeed);
    LeftRear.set(leftSpeed); // Invert direction for LeftRear motor
    RightFront.set(-rightSpeed);
    RightRear.set(rightSpeed); // Invert direction for RightRear motor

    // Joystick Statistics
    SmartDashboard.putNumber("Joystick X", x);
    SmartDashboard.putNumber("Joystick Y", y);
    SmartDashboard.putNumber("Joystick Angle", A);

    // Power Statistics
    SmartDashboard.putNumber("Left Power (%)", leftSpeed);
    SmartDashboard.putNumber("Right Power (%)", rightSpeed);
  }

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    SmartDashboard.putData("PDP", pdp);

    // Configure autonomous options
    m_chooser.setDefaultOption("Autonomous Middle", AutonomousA);
    m_chooser.addOption("Autonomous Left", AutonomousB);
    m_chooser.addOption("Autonomous Right", AutonomousC);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Update SmartDashboard if compressor is actively running
    boolean isCompressorRunning = m_ph.getCompressor();
    SmartDashboard.putBoolean("Compressor Running", isCompressorRunning);

    // Gets the Compressor Current
    SmartDashboard.putNumber("Compressor Current (A)", m_ph.getCompressorCurrent());

    // Get the current state of the solenoid
    DoubleSolenoid.Value solenoidState = m_doubleSolenoid.get();

    // Update the SmartDashboard with a boolean for the solenoid state
    boolean isSolenoidUp = (solenoidState == DoubleSolenoid.Value.kForward);
    SmartDashboard.putBoolean("Solenoid Up", isSolenoidUp);

    // Motor Speed Statistics
    SmartDashboard.putNumber("Left Front Power (%)", LeftFront.get());
    SmartDashboard.putNumber("Left Rear Power (%)", LeftRear.get());
    SmartDashboard.putNumber("Right Front Power (%)", RightFront.get());
    SmartDashboard.putNumber("Right Rear Power (%)", RightRear.get());

    // Drive Motor Temperature Statistics
    SmartDashboard.putNumber("Left Front Temperature (C)", LeftFront.getTemperature());
    SmartDashboard.putNumber("Left Rear Temperature (C)", LeftRear.getTemperature());
    SmartDashboard.putNumber("Right Front Temperature (C)", RightFront.getTemperature());
    SmartDashboard.putNumber("Right Rear Temperature (C)", RightRear.getTemperature());

    // Coral Motor Statistics
    SmartDashboard.putNumber("Coral Motor Power (%)", CoralMotor.get());
    SmartDashboard.putNumber("Coral Motor Temperature (C)", CoralMotor.getMotorTemperature());

    // Winch Statistics
    SmartDashboard.putNumber("Winch Power (%)", Winch.get());

    // Battery Voltage Statistic
    SmartDashboard.putNumber("Battery Voltage (V)", BatteryVoltage);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    LeftFront.set(0);
    LeftRear.set(0);
    RightFront.set(0);
    RightRear.set(0);
    Winch.set(0);
    CoralMotor.set(0);
  }

  private Timer m_autonomousTimer = new Timer(); // Timer to track elapsed time

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    m_autonomousTimer.reset();
    m_autonomousTimer.start();
  }

  @Override
  public void autonomousPeriodic() {
    double elapsedTime = m_autonomousTimer.get(); // Get elapsed time

    // Switch based on selected autonomous mode
    switch (m_autoSelected) {
      case AutonomousA:
        AutonomousA(elapsedTime);
        break;
      case AutonomousB:
        AutonomousB(elapsedTime);
        break;
      case AutonomousC:
      default:
        AutonomousC(elapsedTime);
        break;
    }
  }

  private void AutonomousA(double elapsedTime) {
    // 0 to 1.5 seconds, drive motors forward at 40% speed
    if (elapsedTime >= 0 && elapsedTime < 0.8) {
      LeftFront.set(-0.4);
      LeftRear.set(0.4);
      RightFront.set(-0.4);
      RightRear.set(0.4);
    }
    // 1.5 to 2 seconds, drive motors turn off
    if (elapsedTime >= 0.8 && elapsedTime < 1.5) {
      LeftFront.set(0);
      LeftRear.set(0);
      RightFront.set(0);
      RightRear.set(0);
    }
    // 2 to 2.5 seconds, Coral Motor turns on at 15%
    if (elapsedTime >= 1.5 && elapsedTime < 2) {
      CoralMotor.set(0.15);
    }
    // 2.5 to 3 seconds, Coral Motor turns off
    if (elapsedTime >= 2 && elapsedTime < 2.5) {
      CoralMotor.set(0);
    }
    // Disable all motors after 3 seconds
    else if (elapsedTime >= 2.5) {
      LeftFront.set(0);
      LeftRear.follow(LeftFront);
      RightFront.set(0);
      RightRear.follow(RightFront);
      CoralMotor.set(0);
    }
  }

  private void AutonomousB(double elapsedTime) {
    // 0 to 1.5 seconds, drive motors forward at 40% speed
    if (elapsedTime >= 0 && elapsedTime < 1.2) {
      LeftFront.set(-0.4);
      LeftRear.set(0.4);
      RightFront.set(-0.4);
      RightRear.set(0.4);
    }
    // 1.5 to 2 seconds, drive motors turn off
    if (elapsedTime >= 1.2 && elapsedTime < 1.5) {
      LeftFront.set(0);
      LeftRear.set(0);
      RightFront.set(0);
      RightRear.set(0);
    }
    // Disable all motors after 3 seconds
    else if (elapsedTime >= 1.5) {
      LeftFront.set(0);
      LeftRear.follow(LeftFront);
      RightFront.set(0);
      RightRear.follow(RightFront);
    }
  }

  private void AutonomousC(double elapsedTime) {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // Use the left joystick for angle-based control
    setDriveMotorsFromJoystick(-Controller.getLeftX(), -Controller.getLeftY());

    // Variable to control the Coral motor
    double coralMotorSpeed = 0;
    double winchSpeed = 0;

    double leftTriggerValue = Controller.getLeftTriggerAxis();
    double rightTriggerValue = Controller.getRightTriggerAxis();

    if (leftTriggerValue == 1 && rightTriggerValue == 1) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kForward); // Pneumatics up
    } else {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse); // Pneumatics down
    }

    // A Button: Set Coral Motor forwards 15% (SLOW)
    if (Controller.getAButton()) {
      coralMotorSpeed = 0.15;
    }

    // B Button: Set Coral Motor forwards 26% (FAST)
    if (Controller.getBButton()) {
      coralMotorSpeed = 0.26;
    }

    // Y Button: Reverse coral motor backwards 25%
    if (Controller.getYButton()) {
      coralMotorSpeed = -0.25;
    }

    // Winch goes forward
    if (Controller.getPOV() == 0) {
      winchSpeed = -1;
    }

    // Winch goes down
    if (Controller.getPOV() == 180) {
      winchSpeed = 0.5;
    }

    // Set the motor speed based on the input
    CoralMotor.set(coralMotorSpeed);
    Winch.set(winchSpeed);
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
