/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  WPI_TalonFX left1 = new WPI_TalonFX(1);
  WPI_TalonFX left2 = new WPI_TalonFX(2);
  WPI_TalonFX right1 = new WPI_TalonFX(3);
  WPI_TalonFX right2 = new WPI_TalonFX(4);
  SpeedControllerGroup left = new SpeedControllerGroup(left1, left2);
  SpeedControllerGroup right = new SpeedControllerGroup(right1, right2);
  DifferentialDrive mainDrive = new DifferentialDrive(left, right);

  WPI_TalonSRX fortuneWheel = new WPI_TalonSRX(45);
  WPI_TalonFX lifter = new WPI_TalonFX(10);
  WPI_TalonFX advanceBelt = new WPI_TalonFX(5);
  WPI_TalonFX accelerator = new WPI_TalonFX(6);
  WPI_VictorSPX intake = new WPI_VictorSPX(7);
  WPI_VictorSPX hopper = new WPI_VictorSPX(11);
  WPI_TalonFX leftShooter = new WPI_TalonFX(8);
  WPI_TalonFX rightShooter = new WPI_TalonFX(9);
  SpeedControllerGroup shooter = new SpeedControllerGroup(leftShooter, rightShooter);

  DoubleSolenoid shifter = new DoubleSolenoid(4, 3); //Flip the order of the numbers if this is backwards
  DoubleSolenoid intakePiston = new DoubleSolenoid(2, 5);
  DoubleSolenoid angler = new DoubleSolenoid(1, 6); 
  Value out = Value.kForward;
  Value in = Value.kReverse;

  AnalogInput pressureSensor = new AnalogInput(0);

  Limelight limelight = new Limelight();
  AHRS gyro = new AHRS(Port.kMXP); //NavX
  //CTREEncoder leftDriveEnc = new CTREEncoder(left1, false);
  //CTREEncoder leftShootEnc = new CTREEncoder(leftShooter, false);
  TalonFXSensorCollection leftDriveEnc = left1.getSensorCollection();
  TalonFXSensorCollection leftShootEnc = leftShooter.getSensorCollection();
  TalonFXSensorCollection liftEnc = lifter.getSensorCollection();
  Joystick driver = new Joystick(0);
  Joystick manip = new Joystick(1);
  ////Encoder rotenc = new Encoder(0, 1, false, Encoder.EncodingType.k2X);


  //First term (P): If the robot overshoots too much, make the first number smaller
  //Second term (I): Start 0.000001 and then double until it does stuff
  //Third term (D): To make the adjustment process go faster, start D at like 0.001 and tune
  PIDController turnPID = new PIDController(0.05, 0.08, 0.005);// 0.02, 0.05
  PIDController shooterSpeedPID = new PIDController(0.00003, 0.0002, 0);
  PIDController strPID = new PIDController(0.05, 0, 0);
  PIDController liftPID = new PIDController(0.00001, 0, 0);
  Timer timer = new Timer();
  
  DigitalInput proximity_sensor = new DigitalInput(0);//Infrared sensor
  DigitalInput slot1 = new DigitalInput(2); //Digital inputs for the auton switch
	DigitalInput slot2 = new DigitalInput(3);
  DigitalInput slot3 = new DigitalInput(4);
  AutonType autonMode; //This is used to select which auton mode the robot must do
  int step = 1; //This keeps track of which step of the auton sequence it is in

  final static double lowGear = 8.0*Math.PI*10./545166.;
  boolean onTarget = false;
  double lastTurn = 1;
  double liftPosition = 0.0;
  double liftMax =446314.000000;
  double liftMin =50000.0;

  @Override
  public void robotInit() {
    rightShooter.setInverted(true);
    accelerator.setInverted(true);
    intake.setInverted(true);
    limelight.setPipeline(1);
    liftPID.setSetpoint(liftMin);
    lifter.setInverted(true);

    CameraServer camera = CameraServer.getInstance();
    camera.startAutomaticCapture("cam0", 0);
  }

  @Override
  public void disabledPeriodic() {
    read();
  }

  @Override
  public void robotPeriodic() {
    read();
  }

  @Override
  public void autonomousInit() {
		if(slot1.get()) { //If the first position of the auton switch is selected, choose the leftSide auton
			autonMode = AutonType.leftSide;
		} else if(slot2.get()) { //If the second position of the auton switch is selected, chose the middle auton
			autonMode = AutonType.middle;
		} else if(slot3.get()) { //If the third position of the auton switch is selected, choose the right auton
			autonMode = AutonType.rightSide;
		} else { //If the fourth position of the auton switch is selected, choose to just drive straight
			autonMode = AutonType.straight;
    }

    if(autonMode == null) { //This is an emergency backup operation in case the auton selection failed
			autonMode = AutonType.straight;
			System.out.println("ERROR: autonMode is null. Defaulting to cross auto line.");
    }
    autonMode = AutonType.leftSide;//////////////////////////////////////////////////////////
    limelight.setPipeline(1);
    strPID.setSetpoint(0.0); //Set the drive-straight PID setpoint to 0 degrees
    leftDriveEnc.setIntegratedSensorPosition(0, 20);
    gyro.reset();
    step = 1;
  }

  @Override
  public void autonomousPeriodic() {
    read();
    switch(autonMode) { //This takes the auton mode selected and executes its corresponding function
      case leftSide:
        leftSide();
      break;
      case middle:
        middle();
      break;
      case rightSide:
        rightSide();
      break;
      case straight:
        straight();
      break;
    }
  }

  @Override
  public void teleopPeriodic() {

    if(driver.getRawButtonPressed(1)) {
      limelight.setPipeline(1);
    }
    //DRIVER CONTROLS//
    final double rot = driver.getRawAxis(2);
    final double y = driver.getRawAxis(1);

    if(driver.getRawButton(2)) {
      Interpolator.setOffset(400); //TODO alter this number to a better value
    } else {
      Interpolator.setOffset(0);
    }

    if(!driver.getRawButton(1) && ( Math.abs(rot) >= 0.15 || Math.abs(y) >= 0.15)) {
      mainDrive.curvatureDrive(-y, -rot/2., Math.abs(y) < 0.2);
      accelerator.set(0);
      shooter.set(0.0);
    } else if(driver.getRawButton(1) && limelight.hasValidTarget()) { //If the driver is pulling the trigger and the limelight has a target, go into vision-targeting mode
      System.out.println(limelight.getY());
      turnPID.setSetpoint(0.0); //Set the turning setpoint to 0 degrees
      double rotationSpeed = turnPID.calculate(limelight.getX()); //Calculate turning speed based on the limelight reading
      if(rotationSpeed != 0 && lastTurn != 0) { //Check if the current and last turn speeds are non-zero
        if((int) rotationSpeed/Math.abs(rotationSpeed) != (int) lastTurn/Math.abs(lastTurn)) { //See if the signs are equal to each other (+ or -)
          turnPID.reset(); //If the robot must change its direction, reset its PID
        }
      }
      if(rotationSpeed > 0.6) {
        rotationSpeed = 0.6;
      }
      if(rotationSpeed < -0.6) {
        rotationSpeed = -0.6;
      }
      mainDrive.arcadeDrive(0.0, rotationSpeed); //Turn the robot
      lastTurn = rotationSpeed; //Record the current speed into the previous speed

      shooterSpeedPID.setSetpoint(Interpolator.getInterpolation(limelight.getY())); //TODO: make this change based on distance
      double shooterSpeed = shooterSpeedPID.calculate(leftShootEnc.getIntegratedSensorVelocity()); //Calculate the PID speed
      shooterSpeed = Math.abs(shooterSpeed); //Make sure the wheel only spins forwards
      shooter.set(shooterSpeed); //Power the flywheel
      accelerator.set(1); //Power the accelerator

      double shooterError = Math.abs(leftShootEnc.getIntegratedSensorVelocity() - shooterSpeedPID.getSetpoint()); //Calculate the error
      double turnError = Math.abs(limelight.getX()); //Calculate the error
      if(shooterError < 300.0 && turnError < 1.5) { //If both are in range, signal the drivers
        onTarget = true;
      } else {
        onTarget = false;
      }
    } else {
      mainDrive.arcadeDrive(0, 0);
      shooter.set(0.0);
      accelerator.set(0);
    }
    if(driver.getRawButtonReleased(1)) { //If the driver no longer wants to target...
      turnPID.reset(); //Reset the PIDs and turn the onTarget indicator to false
      shooterSpeedPID.reset();
      onTarget = false;
      //limelight.setPipeline(0);
    }
      

      //MANIP CONTROLS//
    double advancer = manip.getRawAxis(1);
    double hoppermove = manip.getRawAxis(1);

    if(manip.getRawButton(7)) {
      fortuneWheel.set(1);
    } else {
      fortuneWheel.set(0);
    }
    if(manip.getRawButton(8)) {
      fortuneWheel.set(-1);
    } else {
      fortuneWheel.set(0);
    }
    

    if(Math.abs(advancer) >= 0.075) {
      advanceBelt.set(advancer);
      hopper.set(hoppermove);
    } else {
      advanceBelt.set(0);
      hopper.set(0);
    }
    if(manip.getRawButton(1)) {
      intakePiston.set(out);
      intake.set(.5);
    } else {
      intakePiston.set(in);
      intake.set(0);
    }
    if(manip.getRawButton(4)) {
      intake.set(-.5);
    } else {
      intake.set(0);
    }

     double liftSpeed = -manip.getRawAxis(5);
    boolean manualLift = false;
    if(Math.abs(liftSpeed) >= 0.2) {
      liftPosition = -liftEnc.getIntegratedSensorPosition();
      liftPID.setSetpoint(liftPosition);
      manualLift = true;
    }
    if(manualLift) {
      if(liftSpeed > 0 && -liftEnc.getIntegratedSensorPosition() > 350000.000000) {
        manualLift = false;
      } else if(liftSpeed < 0 && -liftEnc.getIntegratedSensorPosition() < 100000.000000) {
        manualLift = false;
      }
    }
    if(manip.getRawButton(3)) liftPID.setSetpoint(liftMax + 7824);
    if(manip.getRawButton(2)) liftPID.setSetpoint(liftMin);
    if(!manualLift) {    
      liftSpeed = liftPID.calculate(-liftEnc.getIntegratedSensorPosition());
    }
    
    lifter.set(liftSpeed);
    


    read(); //Put data onto the SmartDashboard
  }


  void read() {
    SmartDashboard.putNumber("Pressure", pressureSensor.getVoltage()*50.0-25.0);
    SmartDashboard.putNumber("Shooter Encoder Speed", leftShootEnc.getIntegratedSensorVelocity());
    SmartDashboard.putNumber("Drive Encoder", leftDriveEnc.getIntegratedSensorPosition());
    SmartDashboard.putNumber("Limelight X Angle", limelight.getX());
    SmartDashboard.putNumber("Limelight Y Angle", limelight.getY());
    SmartDashboard.putBoolean("On Target", onTarget);
    SmartDashboard.putNumber("Interpolation", Interpolator.getInterpolation(limelight.getY()));
    SmartDashboard.putNumber("Lifter", -liftEnc.getIntegratedSensorPosition());
    SmartDashboard.putNumber("liftstick", -manip.getRawAxis(5));
    SmartDashboard.putNumber("Gyro angle", gyro.getAngle());
  }

  @Override
  public void testInit() {
    limelight.setPipeline(1);
  }
  
  @Override
  public void testPeriodic() {  
  }
  void rightSide() {
    switch(step) {
      case 1:
        if(gyro.getAngle() > -30) {
          mainDrive.arcadeDrive(0, 0.5);
        } else {
          mainDrive.arcadeDrive(0, 0);
          step = 2;
        }
      break;
      case 2:
        if(!onTarget) {
          turnPID.setSetpoint(0.0); //Set the turning setpoint to 0 degrees
          double rotationSpeed = turnPID.calculate(limelight.getX()); //Calculate turning speed based on the limelight reading
          if(rotationSpeed != 0 && lastTurn != 0) { //Check if the current and last turn speeds are non-zero
            if((int) rotationSpeed/Math.abs(rotationSpeed) != (int) lastTurn/Math.abs(lastTurn)) { //See if the signs are equal to each other (+ or -)
              turnPID.reset(); //If the robot must change its direction, reset its PID
            }
          }
          if(rotationSpeed > 0.6) {
            rotationSpeed = 0.6;
          }
          if(rotationSpeed < -0.6) {
            rotationSpeed = -0.6;
          }
          mainDrive.arcadeDrive(0.0, rotationSpeed); //Turn the robot
          lastTurn = rotationSpeed; //Record the current speed into the previous speed

          shooterSpeedPID.setSetpoint(Interpolator.getInterpolation(limelight.getY())); //TODO: make this change based on distance
          double shooterSpeed = shooterSpeedPID.calculate(leftShootEnc.getIntegratedSensorVelocity()); //Calculate the PID speed
          shooterSpeed = Math.abs(shooterSpeed); //Make sure the wheel only spins forwards
          shooter.set(shooterSpeed); //Power the flywheel
          accelerator.set(1); //Power the accelerator

          double shooterError = Math.abs(leftShootEnc.getIntegratedSensorVelocity() - shooterSpeedPID.getSetpoint()); //Calculate the error
          double turnError = Math.abs(limelight.getX()); //Calculate the error
          if(shooterError < 300.0 && turnError < 3.0) { //If both are in range, signal the drivers
            onTarget = true;
          } else {
            onTarget = false;
          }
        } else {
          step = 3;
          timer.start();
        }
      break;
      case 3:
        if(timer.get() < 8) {
          turnPID.setSetpoint(0.0); //Set the turning setpoint to 0 degrees
          double rotationSpeed = turnPID.calculate(limelight.getX()); //Calculate turning speed based on the limelight reading
          if(rotationSpeed != 0 && lastTurn != 0) { //Check if the current and last turn speeds are non-zero
            if((int) rotationSpeed/Math.abs(rotationSpeed) != (int) lastTurn/Math.abs(lastTurn)) { //See if the signs are equal to each other (+ or -)
              turnPID.reset(); //If the robot must change its direction, reset its PID
            }
          }
          if(rotationSpeed > 0.6) {
            rotationSpeed = 0.6;
          }
          if(rotationSpeed < -0.6) {
            rotationSpeed = -0.6;
          }
          mainDrive.arcadeDrive(0.0, rotationSpeed); //Turn the robot
          lastTurn = rotationSpeed; //Record the current speed into the previous speed

          shooterSpeedPID.setSetpoint(Interpolator.getInterpolation(limelight.getY())); //TODO: make this change based on distance
          double shooterSpeed = shooterSpeedPID.calculate(leftShootEnc.getIntegratedSensorVelocity()); //Calculate the PID speed
          shooterSpeed = Math.abs(shooterSpeed); //Make sure the wheel only spins forwards
          shooter.set(shooterSpeed); //Power the flywheel
          accelerator.set(1); //Power the accelerator
          advanceBelt.set(-0.75);
        } else {
          mainDrive.arcadeDrive(0, 0);
          shooter.set(0);
          accelerator.set(0);
          advanceBelt.set(0);
          step = 4;
        }
      break;
      case 4:
        if(gyro.getAngle() < 0) {
          mainDrive.arcadeDrive(0, -0.5);
        } else {
          mainDrive.arcadeDrive(0, 0);
          leftDriveEnc.setIntegratedSensorPosition(0, 20);
          turnPID.reset();
          step = 5;
        }
      break;
      case 5:
        if(getDriveDistance() > -10) {
          double rotationSpeed = turnPID.calculate(gyro.getAngle());
          mainDrive.arcadeDrive(.5, rotationSpeed);
          advanceBelt.set(.50);
          intakePiston.set(out);
        } else {
          mainDrive.arcadeDrive(0, 0);
          advanceBelt.set(0);
          intakePiston.set(in);
          step = 6;
        }
      break;
      case 6:
      if(!onTarget) {
        turnPID.setSetpoint(0.0); //Set the turning setpoint to 0 degrees
        double rotationSpeed = turnPID.calculate(limelight.getX()); //Calculate turning speed based on the limelight reading
        if(rotationSpeed != 0 && lastTurn != 0) { //Check if the current and last turn speeds are non-zero
          if((int) rotationSpeed/Math.abs(rotationSpeed) != (int) lastTurn/Math.abs(lastTurn)) { //See if the signs are equal to each other (+ or -)
            turnPID.reset(); //If the robot must change its direction, reset its PID
          }
        }
        if(rotationSpeed > 0.6) {
          rotationSpeed = 0.6;
        }
        if(rotationSpeed < -0.6) {
          rotationSpeed = -0.6;
        }
        mainDrive.arcadeDrive(0.0, rotationSpeed); //Turn the robot
        lastTurn = rotationSpeed; //Record the current speed into the previous speed

        shooterSpeedPID.setSetpoint(Interpolator.getInterpolation(limelight.getY())); //TODO: make this change based on distance
        double shooterSpeed = shooterSpeedPID.calculate(leftShootEnc.getIntegratedSensorVelocity()); //Calculate the PID speed
        shooterSpeed = Math.abs(shooterSpeed); //Make sure the wheel only spins forwards
        shooter.set(shooterSpeed); //Power the flywheel
        accelerator.set(1); //Power the accelerator

        double shooterError = Math.abs(leftShootEnc.getIntegratedSensorVelocity() - shooterSpeedPID.getSetpoint()); //Calculate the error
        double turnError = Math.abs(limelight.getX()); //Calculate the error
        if(shooterError < 300.0 && turnError < 3.0) { //If both are in range, signal the drivers
          onTarget = true;
        } else {
          onTarget = false;
        }
      } else {
        step = 3;
        timer.start();
      }
    break;
    case 7:
      if(timer.get() < 8) {
        turnPID.setSetpoint(0.0); //Set the turning setpoint to 0 degrees
        double rotationSpeed = turnPID.calculate(limelight.getX()); //Calculate turning speed based on the limelight reading
        if(rotationSpeed != 0 && lastTurn != 0) { //Check if the current and last turn speeds are non-zero
          if((int) rotationSpeed/Math.abs(rotationSpeed) != (int) lastTurn/Math.abs(lastTurn)) { //See if the signs are equal to each other (+ or -)
            turnPID.reset(); //If the robot must change its direction, reset its PID
          }
        }
        if(rotationSpeed > 0.6) {
          rotationSpeed = 0.6;
        }
        if(rotationSpeed < -0.6) {
          rotationSpeed = -0.6;
        }
        mainDrive.arcadeDrive(0.0, rotationSpeed); //Turn the robot
        lastTurn = rotationSpeed; //Record the current speed into the previous speed

        shooterSpeedPID.setSetpoint(Interpolator.getInterpolation(limelight.getY())); //TODO: make this change based on distance
        double shooterSpeed = shooterSpeedPID.calculate(leftShootEnc.getIntegratedSensorVelocity()); //Calculate the PID speed
        shooterSpeed = Math.abs(shooterSpeed); //Make sure the wheel only spins forwards
        shooter.set(shooterSpeed); //Power the flywheel
        accelerator.set(1); //Power the accelerator
        advanceBelt.set(-0.75);
      } else {
        mainDrive.arcadeDrive(0, 0);
        shooter.set(0);
        accelerator.set(0);
        advanceBelt.set(0);
        step = 4;
      }
    break;
    }
  }

  void middle() {
    switch(step) {
      case 1:
        if(gyro.getAngle() < 30) {
          mainDrive.arcadeDrive(0, -0.5);
        } else {
          mainDrive.arcadeDrive(0,0);
          step = 2;
        }
      break;
      case 2:
        if(!onTarget) {
          turnPID.setSetpoint(0.0); //Set the turning setpoint to 0 degrees
          double rotationSpeed = turnPID.calculate(limelight.getX()); //Calculate turning speed based on the limelight reading
          if(rotationSpeed != 0 && lastTurn != 0) { //Check if the current and last turn speeds are non-zero
            if((int) rotationSpeed/Math.abs(rotationSpeed) != (int) lastTurn/Math.abs(lastTurn)) { //See if the signs are equal to each other (+ or -)
              turnPID.reset(); //If the robot must change its direction, reset its PID
            }
          }
          if(rotationSpeed > 0.6) {
            rotationSpeed = 0.6;
          }
          if(rotationSpeed < -0.6) {
            rotationSpeed = -0.6;
          }
          mainDrive.arcadeDrive(0.0, rotationSpeed); //Turn the robot
          lastTurn = rotationSpeed; //Record the current speed into the previous speed

          shooterSpeedPID.setSetpoint(Interpolator.getInterpolation(limelight.getY())); //TODO: make this change based on distance
          double shooterSpeed = shooterSpeedPID.calculate(leftShootEnc.getIntegratedSensorVelocity()); //Calculate the PID speed
          shooterSpeed = Math.abs(shooterSpeed); //Make sure the wheel only spins forwards
          shooter.set(shooterSpeed); //Power the flywheel
          accelerator.set(1); //Power the accelerator

          double shooterError = Math.abs(leftShootEnc.getIntegratedSensorVelocity() - shooterSpeedPID.getSetpoint()); //Calculate the error
          double turnError = Math.abs(limelight.getX()); //Calculate the error
          if(shooterError < 300.0 && turnError < 3.0) { //If both are in range, signal the drivers
            onTarget = true;
          } else {
            onTarget = false;
          }
        } else {
          step = 3;
          timer.start();
        }
        break;
      case 3:
      if(timer.get() < 8) {
        turnPID.setSetpoint(0.0); //Set the turning setpoint to 0 degrees
        double rotationSpeed = turnPID.calculate(limelight.getX()); //Calculate turning speed based on the limelight reading
        if(rotationSpeed != 0 && lastTurn != 0) { //Check if the current and last turn speeds are non-zero
          if((int) rotationSpeed/Math.abs(rotationSpeed) != (int) lastTurn/Math.abs(lastTurn)) { //See if the signs are equal to each other (+ or -)
            turnPID.reset(); //If the robot must change its direction, reset its PID
          }
        }
        if(rotationSpeed > 0.6) {
          rotationSpeed = 0.6;
        }
        if(rotationSpeed < -0.6) {
          rotationSpeed = -0.6;
        }
        mainDrive.arcadeDrive(0.0, rotationSpeed); //Turn the robot
        lastTurn = rotationSpeed; //Record the current speed into the previous speed

        shooterSpeedPID.setSetpoint(Interpolator.getInterpolation(limelight.getY())); //TODO: make this change based on distance
        double shooterSpeed = shooterSpeedPID.calculate(leftShootEnc.getIntegratedSensorVelocity()); //Calculate the PID speed
        shooterSpeed = Math.abs(shooterSpeed); //Make sure the wheel only spins forwards
        shooter.set(shooterSpeed); //Power the flywheel
        accelerator.set(1); //Power the accelerator
        advanceBelt.set(-0.75);
      } else {
        mainDrive.arcadeDrive(0, 0);
        shooter.set(0);
        accelerator.set(0);
        advanceBelt.set(0);
        step = 4;
      }
    break;
    case 4:
    if(gyro.getAngle() > 0) {
      mainDrive.arcadeDrive(0, 0.5);
    } else {
      mainDrive.arcadeDrive(0, 0);
      leftDriveEnc.setIntegratedSensorPosition(0, 20);
      turnPID.reset();
      step = 5;
    }
  break;
  case 5:
    if(getDriveDistance() > -200) {
      double rotationSpeed = turnPID.calculate(gyro.getAngle());
      mainDrive.arcadeDrive(.5, rotationSpeed);
    } else {
      mainDrive.arcadeDrive(0, 0);
      step = 6;
    }
  break;

        }
  }

  void leftSide() {
    
    switch(step) {
      case 1:
        if(gyro.getAngle() < 45) {
          mainDrive.arcadeDrive(0, -0.5);
        } else {
          mainDrive.arcadeDrive(0, 0);
          step = 2;
        }
      break;
      case 2:
        if(!onTarget) {
          turnPID.setSetpoint(0.0); //Set the turning setpoint to 0 degrees
          double rotationSpeed = turnPID.calculate(limelight.getX()); //Calculate turning speed based on the limelight reading
          if(rotationSpeed != 0 && lastTurn != 0) { //Check if the current and last turn speeds are non-zero
            if((int) rotationSpeed/Math.abs(rotationSpeed) != (int) lastTurn/Math.abs(lastTurn)) { //See if the signs are equal to each other (+ or -)
              turnPID.reset(); //If the robot must change its direction, reset its PID
            }
          }
          if(rotationSpeed > 0.6) {
            rotationSpeed = 0.6;
          }
          if(rotationSpeed < -0.6) {
            rotationSpeed = -0.6;
          }
          mainDrive.arcadeDrive(0.0, rotationSpeed); //Turn the robot
          lastTurn = rotationSpeed; //Record the current speed into the previous speed

          shooterSpeedPID.setSetpoint(Interpolator.getInterpolation(limelight.getY())); //TODO: make this change based on distance
          double shooterSpeed = shooterSpeedPID.calculate(leftShootEnc.getIntegratedSensorVelocity()); //Calculate the PID speed
          shooterSpeed = Math.abs(shooterSpeed); //Make sure the wheel only spins forwards
          shooter.set(shooterSpeed); //Power the flywheel
          accelerator.set(1); //Power the accelerator

          double shooterError = Math.abs(leftShootEnc.getIntegratedSensorVelocity() - shooterSpeedPID.getSetpoint()); //Calculate the error
          double turnError = Math.abs(limelight.getX()); //Calculate the error
          if(shooterError < 300.0 && turnError < 3.0) { //If both are in range, signal the drivers
            onTarget = true;
          } else {
            onTarget = false;
          }
        } else {
          step = 3;
          timer.start();
        }
      break;
      case 3:
        if(timer.get() < 8) {
          turnPID.setSetpoint(0.0); //Set the turning setpoint to 0 degrees
          double rotationSpeed = turnPID.calculate(limelight.getX()); //Calculate turning speed based on the limelight reading
          if(rotationSpeed != 0 && lastTurn != 0) { //Check if the current and last turn speeds are non-zero
            if((int) rotationSpeed/Math.abs(rotationSpeed) != (int) lastTurn/Math.abs(lastTurn)) { //See if the signs are equal to each other (+ or -)
              turnPID.reset(); //If the robot must change its direction, reset its PID
            }
          }
          if(rotationSpeed > 0.6) {
            rotationSpeed = 0.6;
          }
          if(rotationSpeed < -0.6) {
            rotationSpeed = -0.6;
          }
          mainDrive.arcadeDrive(0.0, rotationSpeed); //Turn the robot
          lastTurn = rotationSpeed; //Record the current speed into the previous speed

          shooterSpeedPID.setSetpoint(Interpolator.getInterpolation(limelight.getY())); //TODO: make this change based on distance
          double shooterSpeed = shooterSpeedPID.calculate(leftShootEnc.getIntegratedSensorVelocity()); //Calculate the PID speed
          shooterSpeed = Math.abs(shooterSpeed); //Make sure the wheel only spins forwards
          shooter.set(shooterSpeed); //Power the flywheel
          accelerator.set(1); //Power the accelerator
          advanceBelt.set(-0.75);
        } else {
          mainDrive.arcadeDrive(0, 0);
          shooter.set(0);
          accelerator.set(0);
          advanceBelt.set(0);
          step = 4;
        }
      break;
      case 4:
        if(gyro.getAngle() > 0) {
          mainDrive.arcadeDrive(0, 0.5);
        } else {
          mainDrive.arcadeDrive(0, 0);
          leftDriveEnc.setIntegratedSensorPosition(0, 20);
          turnPID.reset();
          step = 5;
        }
      break;
      case 5:
        if(getDriveDistance() > -200) {
          double rotationSpeed = turnPID.calculate(gyro.getAngle());
          mainDrive.arcadeDrive(.5, rotationSpeed);
        } else {
          mainDrive.arcadeDrive(0, 0);
          step = 6;
        }
      break;
    } 
  }

  void straight() { //Drive [leaveDistance] inches - figure out how far the robot must move to leave the line (you get points)
    double leaveDistance = -14;
    switch(step) { //This takes in which step of the auton sequence and executes its correlated actions
      case 1: //Step 1:
        if(getDriveDistance() > leaveDistance) { //If the robot has driven less than [leaveDistance]...
          double turnSpeed = strPID.calculate(gyro.getAngle()); //Use the drive-straight PID to calculate its turn speed
          mainDrive.arcadeDrive(-0.6, turnSpeed); //Drive at 0.3 speed forwards and correct its heading with the turnSpeed
        } else {//If the robot has driven past [leaveDistance]
          step = 2; //Set the step to 2. Because there is no step 2 programmed, this will stop the auton (which is a good thing)
          mainDrive.arcadeDrive(0, 0); //Stop the robot
        }
      break;
    }
  }

  double getDriveDistance() { //This function uses the drive encoder to calculate how far the robot has driven
    return leftDriveEnc.getIntegratedSensorPosition() * lowGear;
  }

  enum AutonType {//A list of different auton types
		rightSide, middle, leftSide, straight
	}
}

/**
 * 1) Shooting code: Set the shooter motors to 1.0 speed and look at the
 *    encoder velocity on the SmartDashboard. This will essentially be
 *    the fastest encoder velocity possible. Use this function- 
 *    "shooterSpeedPID.setSetpoint(measuredSpeed/2.);". You can then begin
 *    the PID tuning process. Use the SmartDashboard to see if the speed
 *    is crazy or stable. If it's crazy, change the value of P in the PID
 *    thing at the top of the code. If the motor seems like it isn't 
 *    moving as fast as it should, increase P.
 * 
 * 2) Auton code: We need to know the gear ratio of the drive motors. This
 *    is important because the robot needs to be able to calculate how many
 *    inches it has driven. You will also need to know the diameter of the
 *    drive wheels for the same purpose. Write them here:
 *    Gear ratio:              (Note: this should look like 1:50 [or whatever number it is])
 *    Wheel diameter:
 */