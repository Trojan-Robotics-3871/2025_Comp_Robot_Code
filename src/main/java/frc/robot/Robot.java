// FRC Imports - DO NOT REMOVE!
package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Venom Imports
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

public class Robot extends TimedRobot {

  // Tank drive wheels -
  CANVenom LeftRear = new CANVenom(1); // Serial 6701
  CANVenom RightRear = new CANVenom(2); // Serial 6739
  CANVenom LeftFront = new CANVenom(3); // Serial 6682
  CANVenom RightFront = new CANVenom(4); // Serial 6725

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
    double maxSpeed = 0.7;
    leftSpeed = Math.max(-maxSpeed, Math.min(leftSpeed, maxSpeed));
    rightSpeed = Math.max(-maxSpeed, Math.min(rightSpeed, maxSpeed));

    // Set motor speeds and invert direction for follower motors
    LeftFront.set(leftSpeed);
    LeftRear.set(-leftSpeed); // Invert direction for LeftRear motor
    RightFront.set(rightSpeed);
    RightRear.set(-rightSpeed); // Invert direction for RightRear motor
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
    // Use the right joystick for angle-based control
    setDriveMotorsFromJoystick(Controller.getRightX(), -Controller.getRightY());
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
