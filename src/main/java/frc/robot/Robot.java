// FRC Imports - DO NOT REMOVE!
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Venom Imports -
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;

// REV Imports -
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Robot extends TimedRobot {

  private PowerDistribution BatteryVolt = new PowerDistribution();

  // Tank drive wheels -
  CANVenom LeftRear = new CANVenom(1); // Serial 6701
  CANVenom RightRear = new CANVenom(2); // Serial 6739
  CANVenom LeftFront = new CANVenom(3); // Serial 6682
  CANVenom RightFront = new CANVenom(4); // Serial 6725

  // Spark Motors -
  SparkMax CoralMotor = new SparkMax(5, MotorType.kBrushless);

  // Xbox Controller Configuration -
  XboxController Controller = new XboxController(0);

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

  // Create drive motors, configured for forward/backwards
  public void setDriveMotors(double forward, double backward) {

    // Calculate the raw left and right values
    double left = forward - backward;
    double right = forward + backward;

    // Configuration for modifying drive wheels speed
    double maxSpeed = 0.5;
    double leftSpeed = Math.max(-maxSpeed, Math.min(left, maxSpeed));
    double rightSpeed = Math.max(-maxSpeed, Math.min(right, maxSpeed));

    // Set motor speeds
    LeftFront.set(-leftSpeed);
    RightFront.follow(LeftFront);
    LeftRear.set(-rightSpeed);
    RightRear.follow(LeftRear);

    // Coral Motor Stats for SmartDashboard
    SmartDashboard.putNumber("Coral Motor (%)", CoralMotor.get());

    // Create battery configuration for SmartDashboard
    double BatteryVoltage = BatteryVolt.getVoltage();

    // Battery Voltage statistic
    SmartDashboard.putNumber("Battery Voltage (V)", BatteryVoltage);

    // Power Speeds
    SmartDashboard.putNumber("Left Power (%)", leftSpeed);
    SmartDashboard.putNumber("Right Power (%)", rightSpeed);

    // Motor speed stats
    SmartDashboard.putNumber("Left Rear", LeftRear.get());
    SmartDashboard.putNumber("Right Rear", RightRear.get());
    SmartDashboard.putNumber("Left Front", LeftFront.get());
    SmartDashboard.putNumber("Right Front", RightFront.get());
  }

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    LeftFront.setBrakeCoastMode(BrakeCoastMode.Brake);
    LeftRear.follow(LeftFront);
    RightFront.setBrakeCoastMode(BrakeCoastMode.Brake);
    RightRear.follow(RightFront);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    double rightX = Controller.getRightX();
    double leftY = Controller.getLeftY();

    // Set the drive motors
    setDriveMotors(rightX, leftY);

    // Variable to control the Coral motor
    double coralMotorSpeed = 0;

    // Y Button: intake out, floor motor reverse
    if (Controller.getAButton()) {
      coralMotorSpeed = -0.15;
    }

    // Set the motor speed based on the input
    CoralMotor.set(coralMotorSpeed);

    // Display joystick x and y values on the SmartDashboard
    SmartDashboard.putNumber("Joystick X", rightX);
    SmartDashboard.putNumber("Joystick Y", leftY);

    // Calculate and display the joystick angle on the SmartDashboard
    double joystickAngle = getJoystickAngle(rightX, leftY);
    SmartDashboard.putNumber("Joystick Angle", joystickAngle);
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
