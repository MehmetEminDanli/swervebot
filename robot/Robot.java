// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.FarnebackOpticalFlow;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.fasterxml.jackson.databind.node.BooleanNode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.autonomous.AutoBalance;

import java.lang.annotation.Retention;
import java.sql.Time;
import java.util.Map;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

//import frc.robot.subsystems.PneumaticSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  XboxController m_stick = new XboxController(0);
  Joystick ikincJoystick = new Joystick(1);
  static WPI_VictorSPX dikeymotor1 = new WPI_VictorSPX(10);
  static WPI_VictorSPX dikeymotor2 = new WPI_VictorSPX(11);
  WPI_VictorSPX intakeyukari = new WPI_VictorSPX(12);
  WPI_VictorSPX intakeileri = new WPI_VictorSPX(13);
  // Compressor komprasor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  DigitalInput enalt = new DigitalInput(1);
  DigitalInput enust = new DigitalInput(0);
  DigitalInput intakeilerisensor = new DigitalInput(2);
  DigitalInput intakegerisensor = new DigitalInput(3);
  DigitalInput intakeyukarisensor = new DigitalInput(4);
  DigitalInput ortasensor = new DigitalInput(5);
  DigitalInput topalma1 = new DigitalInput(6);
  DigitalInput topalma2 = new DigitalInput(7);
  DigitalInput topalma3 = new DigitalInput(8);

  Solenoid topSolenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
  Solenoid topSolenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
  Solenoid intakeilerSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
  Solenoid deneme = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  public boolean a = false;
  Boolean b=false;

  Timer m_Timer = new Timer();
  Timer m_Timer2 = new Timer();
  Timer timer3 = new Timer();
  private RobotContainer m_robotContainer;

  boolean buttonlock = false;
  Timer ilerlemeTimer = new Timer();

  // private PneumaticSubsystem pneumaticSubsystem;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    Thread m_visionThread;
    m_visionThread = new Thread(
        () -> {
          // Get the UsbCamera from CameraServer
          UsbCamera camera = CameraServer.startAutomaticCapture();
          // Set the resolution
          camera.setResolution(480, 300);
          camera.setFPS(30);

          // Get a CvSink. This will capture Mats from the camera
          CvSink cvSink = CameraServer.getVideo();
          // Setup a CvSource. This will send images back to the Dashboard
          CvSource outputStream = CameraServer.putVideo("Rectangle", 100, 100);

          // Mats are very memory expensive. Lets reuse this Mat.
          Mat mat = new Mat();

          // This cannot be 'true'. The program will never exit if it is. This
          // lets the robot stop this thread when restarting robot code or
          // deploying.
          while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat. If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
              // Send the output the error.
              outputStream.notifyError(cvSink.getError());
              // skip the rest of the current iteration
              continue;
            }
            // Put a rectangle on the image
            Imgproc.rectangle(
                mat, new Point(1000, 1000), new Point(300, 300), new Scalar(255, 255, 255), 1);
            // Give the output stream a new image to display
            outputStream.putFrame(mat);
          }
        });
    m_visionThread.setDaemon(true);
    m_visionThread.start();

    // Run compressor with in start
    // pneumaticSubsystem = new PneumaticSubsystem();
    // pneumaticSubsystem.compOn();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    RobotContainer.poseEstimation.periodic();
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */

  @Override
  public void autonomousInit() {
    intakeilerSolenoid.set(true);
    a = false;
    timer3.reset();


    /*
     * m_autonomousCommand = m_robotContainer.getAutonomousCommand();
     * 
     * if (m_autonomousCommand != null) {
     * m_autonomousCommand.schedule();
     * }
     */

    // schedule the autonomous command (example)
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    ilerlemeTimer.start();

    // if(ilerlemeTimer.get() < 1){
    // double ty = MathUtil.applyDeadband(0, 0.1);
    // double tx = MathUtil.applyDeadband(0.4, 0.1);
    // double r = MathUtil.applyDeadband(0, 0.1);

    // double vx = tx ;
    // double vy = ty ;
    // double omega = r ;

    // ChassisSpeeds fieldRelSpeeds = new ChassisSpeeds(vx, vy, omega);

    // RobotContainer.drivetrain.drive(fieldRelSpeeds);
    // } else {
    // double ty = MathUtil.applyDeadband(0, 0.1);
    // double tx = MathUtil.applyDeadband(0, 0.1);
    // double r = MathUtil.applyDeadband(0, 0.1);

    // double vx = tx * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    // double vy = ty * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    // double omega = r * DriveConstants.MAX_ANGULAR_SPEED;

    // ChassisSpeeds fieldRelSpeeds = new ChassisSpeeds(vx, vy, omega);

    // RobotContainer.drivetrain.drive(fieldRelSpeeds);
    // }

    if (a == false) {
      if (ortasensor.get() == true) {
        dikeymotor1.set(-1);
      } else {
        dikeymotor1.stopMotor();
        a = true;
        timer3.reset();
        timer3.start();
      }
    }
    /*
     * else if(intakeilerisensor.get()==true && timer3.get()<2){
     * intakeileri.set(0.65);
     * 
     * }
     */
    else if (timer3.get() > 2.0 & timer3.get() < 4.0) {
      intakeileri.stopMotor();
      intakeyukari.set(1);
    } else if (timer3.get() > 4.0 & timer3.get() < 4.5) {
      intakeyukari.stopMotor();
    }

    else if (timer3.get() > 4.5 && timer3.get() > 5.2) {

      intakeyukari.stopMotor();
      topSolenoid1.set(false);
      topSolenoid2.set(true);
    }
    else if(timer3.get()>5.2&&timer3.get()<6.0){
      if (b == false) {
        if (intakeyukarisensor.get() == true) {
          intakeyukari.set(-0.50);
        } else {
          dikeymotor1.stopMotor();
          b = true;
        }
      }
    }
   
   else {
      dikeymotor1.stopMotor();
      dikeymotor2.stopMotor();
      intakeileri.stopMotor();
      intakeyukari.stopMotor();
    }
  }

  /**
   * m_Timer.start();
   * 
   * m_Timer2.start();
   * 
   * 
   * 
   * while(enust.get()==true){
   * dikeymotor1.set(-1);
   * }
   * 
   * 
   * if(m_Timer2.get()>5&m_Timer2.get()<6.5){
   * intakeyukari.set(1);
   * }
   * else{
   * intakeyukari.set(0);
   * 
   * intakeyukari.setNeutralMode(NeutralMode.Brake);
   * }
   */

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.-
  }

  /** This function is called periodically during operator control. */
  public void yukari() {
    Timer timer2 = new Timer();
    timer2.reset();
    timer2.start();
    intakeilerSolenoid.set(true);
    // while(ortasensor.get()==false)
  }

  @Override
  public void teleopPeriodic() {

    double ty = MathUtil.applyDeadband(RobotContainer.joystick1.getRawAxis(0), 0.1);
    double tx = MathUtil.applyDeadband(-RobotContainer.joystick1.getRawAxis(1), 0.1);
    double r = MathUtil.applyDeadband(RobotContainer.joystick1.getRawAxis(4), 0.1);

    double vx = tx * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double vy = ty * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double omega = r * DriveConstants.MAX_ANGULAR_SPEED;

    ChassisSpeeds fieldRelSpeeds = new ChassisSpeeds(vx, -vy, -omega);

    RobotContainer.drivetrain.drive(fieldRelSpeeds);

    /*
     * Timer m_Timer =new Timer();
     * m_Timer.start();
     * if(m_Timer.get()<1){
     * intakeilerSolenoid.set(true);
     * }
     * intakeilerSolenoid.set(false);
     */

    /* ------------ASANSOR AŞAĞIYA İNME---------- */

    if (enalt.get() == true && m_stick.getRawAxis(2) == 1) {
      new Thread(() -> {
        while (enalt.get() == true) {
          dikeymotor1.set(1);
          dikeymotor2.set(1);

        }
      }).start();

    }

    if (enalt.get() == true && enust.get() == false) {
      new Thread(() -> {
        dikeymotor1.set(-0.10);
        dikeymotor2.set(-0.10);
      }).start();

    }
    /*----------------ASANSÖR YUKARI ÇIKMA ---------------------------- */

    if (ortasensor.get() == true && m_stick.getRawButton(5) == true) {
      new Thread(() -> {
        while (ortasensor.get() == true) {
          dikeymotor1.set(-1);
          dikeymotor2.set(-1);
        }

      }).start();
      System.out.println("slkskllks");

    }

    else {

      dikeymotor1.set(0);

      dikeymotor2.set(0);
      dikeymotor1.setNeutralMode(NeutralMode.Brake);
      dikeymotor2.setNeutralMode(NeutralMode.Brake);

    }
    /*--------------------İNTAKE GERİ GELME KODU--------- */

    if (intakegerisensor.get() == true && m_stick.getRawButton(1) == true) {
      new Thread(() -> {
        while (intakegerisensor.get() == true) {
          intakeileri.set(0.70);
        }
      }).start();

    }
    /* -------------İNTAKE İLERİ GİTME KODU------------------ */

    else if (intakeilerisensor.get() == true && m_stick.getRawButton(4) == true) {
      new Thread(() -> {
        while (intakeilerisensor.get() == true) {
          intakeileri.set(-0.70);
        }
      }).start();

    }

    else {
      intakeileri.set(0);

      intakeileri.setNeutralMode(NeutralMode.Brake);
    }
    /* ---------------İNTAKE YUKARI ÇIKMA KODU-------------- */

    if (intakeyukarisensor.get() == true && m_stick.getRawButton(6) == true) {
      new Thread(() -> {
        while (intakeyukarisensor.get() == true) {
          intakeyukari.set(-1);
        }

      }).start();

    }
    /*-----------------İNTAKE AŞAĞI İNME KODU-------------------- */
    else if (m_stick.getRawAxis(3) == 1) {
      new Thread(() -> {
        intakeyukari.set(1);
      }).start();

    }

    else {
      intakeyukari.set(0);

      intakeyukari.setNeutralMode(NeutralMode.Brake);
    }

    //////////////////// İNTAKE////////////////////
    /*---------------------------ÇENE MANUAL AÇMA KODU-------------- */
    if (m_stick.getRawButton(2) == true) {
      topSolenoid1.set(true);
    }
    /*--------------------ÇENE MANUAL AÇMA KODU------------- */
    else if (m_stick.getRawButton(3) == true) {
      topSolenoid1.set(false);
      topSolenoid2.set(true);
      System.out.println("deneme");

    } else {
      topSolenoid1.set(false);
      topSolenoid2.set(false);
    }
    /*-------------------ÇENE OTOMATİK KAPANMA KODU------------- */
    if (m_stick.getRawButton(3) == false & topalma3.get() == false) {
      topSolenoid1.set(true);

    }
    /*-------------------ÇENE AÇILMA KODU-------------------+-- */
    else if (m_stick.getRawButton(3) == true & topalma3.get() == false) {
      System.out.println("cene acilsddsadsd%di");

      topSolenoid1.set(false);
      topSolenoid2.set(true);
    }

    /*------------ÇENE İLERİ PİSTON AÇMA ---------- */
    if (ikincJoystick.getRawButton(3) == true) {
      intakeilerSolenoid.set(true);
    } else if (ikincJoystick.getRawButton(4) == true) {
      intakeilerSolenoid.set(false);
    }

    /*----------İKİNCİ KOL-------------- */

    /*-------------MANUEL KOL AÇMA--------------    */
    if (ikincJoystick.getRawButton(1) == true) {
      topSolenoid1.set(false);
      topSolenoid2.set(true);
      System.out.println("deneme");

    }

    if (ikincJoystick.getRawButton(3) == true) {
      intakeilerSolenoid.set(false);
    }
    if (ikincJoystick.getRawButton(4) == true) {
      intakeilerSolenoid.set(true);
    }

    /*---------------İNTAKE ASA İNME--------------- */
    if (ikincJoystick.getRawButton(7) == true) {
      intakeyukari.set(1);
    }

    /*----------------İNTAKE AÇISAL YUKARI ÇIKMA------------- */
    if (ikincJoystick.getRawButton(8) == true && intakeyukarisensor.get() == true) {
      intakeyukari.set(-1);
    }
    /*-------------------İNTAKE OTAMATİK AÇISAL YUKARI------- */

    if (intakeyukarisensor.get() == true && ikincJoystick.getRawButton(11) == true) {
      while (intakeyukarisensor.get() == true) {
        intakeyukari.set(-1);
      }
    }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
