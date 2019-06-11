package org.minutebots;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private Spark leftFront = new Spark(4),
          leftBack = new Spark(1),
          rightFront = new Spark(2),
          rightBack = new Spark(3),
          servo = new Spark(0),
          climber = new Spark(5);
  private SendableChooser<Boolean> robotCentric = new SendableChooser<>();
  private Joystick primaryStick = new Joystick(0);
  private MecanumDrive mecanumDrive = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
  private AHRS navX = new AHRS(SPI.Port.kMXP);

  public void robotInit() {
    mecanumDrive.setDeadband(0.18); //Sets deadzone for Mecanum Drive
    robotCentric.addOption("Robot-Centric", true);
    robotCentric.setDefaultOption("Field-Centric", false);
    SmartDashboard.putData("Drive Mode: ", robotCentric);
    liveWindow();
  }

  public void teleopPeriodic() {
    drive();
    climber();
    gearSlide();
    angleAdjustment();
  }

  private void liveWindow() {
    SmartDashboard.putData("NavX", navX);
    LiveWindow.add(mecanumDrive);
    LiveWindow.add(servo);
    LiveWindow.add(climber);
    LiveWindow.add(leftFront);
    LiveWindow.add(leftBack);
    LiveWindow.add(rightFront);
    LiveWindow.add(rightBack);
  }

  //--------------------------------------------Robot Methods------------------------------------------//
  private void drive() {
    double turnThrottle, gyroAngle;
    double speedMultiplier = ((primaryStick.getThrottle() + 1) / 2);
    if (primaryStick.getPOV() == -1) {
      if (primaryStick.getRawButton(1)) {
        turnThrottle = 0.6 * primaryStick.getTwist();
      } else {
        turnThrottle = 0.0;
      }
    } else {
      turnThrottle = limit(angleDifference(navX.getYaw(), primaryStick.getPOV()) * 0.02, 0.4);
    }
    if (!robotCentric.getSelected()) gyroAngle = -navX.getAngle();
    else gyroAngle = 0.0;
    mecanumDrive.driveCartesian(primaryStick.getX() * speedMultiplier, -primaryStick.getY() * speedMultiplier, turnThrottle * 2 * speedMultiplier, gyroAngle);
  }

  private void gearSlide() {
    //Left buttons on the joystick
    //If both buttons are pressed, go to middle value.
    //If upper button is pressed, go to the upper position... ect....
    if (primaryStick.getRawButton(5) && primaryStick.getRawButton(3)) {
      servo.set(0.48);
    } else if (primaryStick.getRawButton(5)) {
      servo.set(0.73);
    } else if (primaryStick.getRawButton(3)) {
      servo.set(0.35);
    }
  }

  private void climber() {
    //Right Buttons on the joystick
    if (primaryStick.getRawButton(6)) {
      climber.set(1);
    } else if (primaryStick.getRawButton(4)) {
      climber.set(-0.3);
    } else {
      climber.set(0);
    }
  }

  private void angleAdjustment() {
    if (primaryStick.getRawButton(11)) {
      navX.setAngleAdjustment(navX.getAngleAdjustment() + 1.125);
    }
    if (primaryStick.getRawButton(12)) {
      navX.setAngleAdjustment(navX.getAngleAdjustment() - 1.125);
    }
    if (primaryStick.getRawButton(10)) {
      navX.reset();
    }
  }

  //--------------------------------------------Utility Methods---------------------------------------//
  private static double limit(double value, double limit) {
    if (value > limit) {
      return limit;
    } else if (value < -limit) {
      return -limit;
    } else {
      return value;
    }
  }

  private static double angleDifference(double startingAngle, double desiredAngle) {
    double difference;
    difference = startingAngle - desiredAngle;
    if (difference > -180 && difference <= 180)
      return -difference;
    else if (difference <= -180)
      return -(difference + 360);
    else if (difference > 180)
      return -(difference - 360);
    else
      return 0;
  }
}