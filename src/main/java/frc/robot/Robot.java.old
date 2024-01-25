package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Joystick;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;


public class Robot extends TimedRobot {


  TalonFX leftDrive1;
  TalonFX leftDrive2;
  TalonFX rightDrive1;
  TalonFX rightDrive2;

  TalonFX liftMotor;
  TalonFX liftMotorSlave;

  TalonSRX armWheels;
  TalonSRX armMotor;
  TalonFX armRotator;

  Joystick driveStick;
  Joystick operatorStick;

  AHRS navX;
  
  boolean isSticking;

  double angleP;
  double angleD;
  double roll;

  boolean brakeStatus;
  boolean armWheelsOn;

  Compressor compressor;
  Solenoid driveGearShiftSolenoid; //exitflag is gone :)
  Solenoid gripperSolenoid; //exitflag is gone :)
  boolean compressorState = false;
  boolean driveShiftBool = true;
  boolean gripperBool = true;
  boolean compState = true;

  //manual mode stuff
  boolean manualMode = true;

  //driving stuff
  boolean fastDriving = false;

  //setting the zero for the arm
  Timer timer;
  boolean zeroCompleted = false;
  boolean zeroSettingBOOL = false;
  double zeroSettingAmperage = 8;
  double zeroSettingTime = 0.2;
  double zeroSettingTimeFlag;

  double armMax;

  double armGoal = 0;
  double armMaxDistance = 19000; //from testing
  double maxLiftSpeedDown = -0.1;
  double maxLiftSpeedUp = 0.3;

  //this is the increment that the arm will go out by each time you click the button. ex 0 -> 10 -> 20 -> 10
  double armGoalIncrement = 0.10;
  double liftGoalIncrement = 0.10;
  
  //lift pid 
  double time;
  double power;
  double shooterFlag;
  private double liftMaxPosition;
  private double armMaxPosition;
  private double liftP;
  private double liftI;
  private double liftD;
  private double liftSetpointPos;

  //Collection<TalonFX> _fxes =  { new TalonFX(1), new TalonFX(2), new TalonFX(3) };

  //auto Variables
  int autoStep = 0;

  @Override
  public void robotInit() {
    leftDrive1 = new TalonFX(1);
    leftDrive2 = new TalonFX(2);
    rightDrive1 = new TalonFX(3);
    rightDrive2 = new TalonFX(4);
    liftMotor = new TalonFX(5);
    liftMotorSlave = new TalonFX(6);

    liftMotorSlave.setInverted(true);

    //testMotor = new TalonSRX(24); //this was on the comp bot for the rotating arm
    armMotor = new TalonSRX(11);
    armMotor.setInverted(true);
    armMotor.setSensorPhase(true);

    armRotator = new TalonFX(50);
    armRotator.setInverted(true);

    armWheels = new TalonSRX(24);

    rightDrive1.setInverted(true);
    rightDrive2.setInverted(true);

    leftDrive2.set(ControlMode.Follower, 1);
    rightDrive2.set(ControlMode.Follower, 3);

    leftDrive1.setNeutralMode(NeutralMode.Coast);
    leftDrive2.setNeutralMode(NeutralMode.Coast);
    rightDrive1.setNeutralMode(NeutralMode.Coast);
    rightDrive2.setNeutralMode(NeutralMode.Coast);

    leftDrive1.setSensorPhase(true);
    rightDrive1.setSensorPhase(true);

    leftDrive1.config_kP(0, 0.01);
    leftDrive1.config_kI(0, 0.0003);
    leftDrive1.config_kD(0, 1);

    rightDrive1.config_kP(0, 0.01);
    rightDrive1.config_kI(0, 0.0003);
    rightDrive1.config_kD(0, 1);



    leftDrive1.configClosedLoopPeakOutput(0, 1);
    rightDrive1.configClosedLoopPeakOutput(0, 1);

    driveStick = new Joystick(0);
    operatorStick = new Joystick(1);

    navX = new AHRS();

    navX.calibrate();

    isSticking = false;

    brakeStatus = true;

    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    driveGearShiftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    gripperSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    SmartDashboard.putNumber("angleP", angleP);
    SmartDashboard.putNumber("angleD", angleD);

    timer = new Timer();
    timer.start();
    //lft PID
    liftP = 0.02;
    liftMotor.config_kP(0, liftP);
    // liftI = 0.000002;
    liftMotor.config_kI(0, 0);
    // liftD = 23.0;
    liftMotor.config_kD(0, 0);
    // liftMotor.configOpenloopRamp(0.15);
    // liftMotor.setSelectedSensorPosition(0);
    // liftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 8, 7, 0.05));
    // liftMotor.configClosedLoopPeakOutput(0, 0.5);
    
    // end of pid*/
    liftMaxPosition = -800000;
    armMaxPosition = 20000;

        //lift pid
    SmartDashboard.putNumber("Lift P", liftP);
    SmartDashboard.putNumber("Lift I", liftI);
    SmartDashboard.putNumber("Lift D", liftD);

    SmartDashboard.putNumber("Arm PID Increment %", 10);

    SmartDashboard.putNumber("ArmP", 0.181);    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    //SmartDashboard.putNumber("shooter power (0-1)", 1);
    //SmartDashboard.putNumber("shooter runtime", 2); :)
    power = SmartDashboard.getNumber("shooter power (0-1)", 1);
    time = SmartDashboard.getNumber("shooter runtime", 2);

    angleP = SmartDashboard.getNumber("angleP", 0.00001);
    angleD = SmartDashboard.getNumber("angleD", 0);
    roll = navX.getRoll();

    SmartDashboard.putBoolean("BreakStatus", brakeStatus);
    SmartDashboard.putNumber("pitch", roll);
    SmartDashboard.putBoolean("sticking?", isSticking);

    SmartDashboard.putNumber("Arm Length POS", armMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Length AMP", armMotor.getStatorCurrent());
    SmartDashboard.putNumber("Arm Length MAX", armMax);
    SmartDashboard.putNumber("Arm length applied", armMotor.getMotorOutputPercent());

    SmartDashboard.putNumber("Arm Rotation Position", armRotator.getSelectedSensorPosition()); //400,000 to 0
    SmartDashboard.putNumber("Lift Position", liftMotor.getSelectedSensorPosition()); //-800,000 to 0

    SmartDashboard.putNumber("Arm PID GOAL", armGoal);
    armGoalIncrement = SmartDashboard.getNumber("Arm PID Increment %", 10)/100;
    liftGoalIncrement = SmartDashboard.getNumber("Lift PID Increment %", 10)/100;

    SmartDashboard.putBoolean("ManualMode", manualMode);
    if(driveStick.getRawButtonPressed(13)){
      manualMode = !manualMode;
    }

    SmartDashboard.putNumber("LeftDriveVEL", leftDrive1.getSelectedSensorVelocity());
    SmartDashboard.putNumber("RightDriveVEL", rightDrive1.getSelectedSensorVelocity());

    //SmartDashboard.putNumber("ArmP", 0);

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    
    armMotor.setSelectedSensorPosition(0);

    armRotator.setSelectedSensorPosition(0);

    liftMotor.setSelectedSensorPosition(0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /* POTNENTIAL AUTO STEPS
     * Placing a block
     *  - Don't place a block (1)
     *  - bottom layer: push it in, regardless of piece type (2)
     *  - mid layer with cone: move lift up, extend arm a bit and release claw (3)
     *  - mid layer with cube: move lift up, extend arm a bit and drive arm wheels out (4)
     *  - top layer with cone: move lift up, rotate arm up, extend arm and release claw (5)
     *  - top layer with cube: move lift up, rotate arm up, extend arm and drive arm wheels out (6)
     * Moving out of community
     *  - Starting in middle of field: drive over the switch, then come back to balance (1)
     *  - starting near sides of field: drive around switch and come back to balance (2)
     * Balancing
     *  - Balance (1)
     *  - Don't balance, teammate is 1114 (2)
     * 
     * - We can create different combinations of auto steps, and call them based on the number 
     * (ex. 2-2-1 auto is scoring a bottom layer piece, driving around the switch and then balancing from the other side)
     */
    switch(autoStep){
      case 0:
        if (armRotator.getSelectedSensorPosition() < 190000){
          armRotator.set(ControlMode.Position, 200000);
          liftMotor.set(ControlMode.Position, -40000);
          armMotor.set(ControlMode.PercentOutput, -.4);
          gripperSolenoid.set(true);
        }
        else{
          autoStep++;

        }
        break;
      case 1:
        armMotor.set(ControlMode.Position, 7360);
        autoStep++;
        break;
      case 2:
        if(Math.abs(armMotor.getSelectedSensorPosition()) < 250) {
          autoStep++;
        }
        break;
      case 3:
        gripperSolenoid.set(false);
        autoStep++;
        break;
      case 4:
        //drive backwards
        leftDrive1.set(ControlMode.Velocity, 3000);
        rightDrive1.set(ControlMode.Velocity, 3000);
        break;
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    armRotator.setSelectedSensorPosition(0);
    liftMotor.setSelectedSensorPosition(0);
    double armP = SmartDashboard.getNumber("ArmP", 0);

    armMotor.config_kP(0, 0.02);
    armMotor.config_kD(0, 0);
    brakeStatus = false;
    setBrakeMode();

    armMotor.setSelectedSensorPosition(0);


    armRotator.setSelectedSensorPosition(0);
    armRotator.config_kP(0, 0.01);
    armRotator.config_kI(0, 0.0);
    armRotator.config_kD(0, 0.0);


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if(driveStick.getRawButton(30)){
      
      if(Math.abs(navX.getRoll()) >= 3){
        isSticking = true;
      } 
      
      if(isSticking){
        balancingPID(-0.016, angleD, 0.4);
      } else {
        leftDrive1.set(ControlMode.PercentOutput, 0.4);
        rightDrive1.set(ControlMode.PercentOutput, 0.4);
      }
    }
    else {
      //drive();
      driveButBetter();
      isSticking = false;
      if (brakeStatus && !isSticking){
        brakeStatus = false;
        //setBrakeMode();
      }

    }

    //if(driveStick.getRawButtonPressed(10)){
      //fastDriving = !fastDriving;
    //}
    ////////////////////////
    //pneumatics 
    /////////////

    
    if(driveStick.getRawButtonPressed(10)){
      driveShiftBool = !driveShiftBool;
    }
    driveGearShiftSolenoid.set(driveShiftBool);


    if(driveStick.getRawButtonPressed(2)){
      gripperBool = !gripperBool;
    }
    gripperSolenoid.set(gripperBool);
     
    // if(driveStick.getRawButtonPressed(3)){
    //     if(compState == false){
    //      compressor.disable();
    //      }
    //      else if(compState == true){
    //        compressor.enableDigital();
    //      } 
    //      compState = !compState;  
    // }
    if(driveStick.getRawButtonPressed(3)){
      shooterFlag = timer.get() + time;
    }
    if(driveStick.getRawButton(3)){ //out
      shooter();
      // armWheels.set(ControlMode.PercentOutput, -1);
      armWheelsOn = false;
     }else if(driveStick.getRawButton(4)){ //in a lot
      armWheels.set(ControlMode.PercentOutput, 0.8);
      armWheelsOn = false;
     }else if(driveStick.getRawButton(1)){ //in a little bit
        armWheels.set(ControlMode.PercentOutput, 0.25);
        armWheelsOn = true;
     }else{
      if (armWheelsOn){
        armWheels.set(ControlMode.PercentOutput, 0.25);
      }
        else{
          armWheels.set(ControlMode.PercentOutput, 0);
        }
      }
    //}if(operatorStick.getRawButton(8)){
    //   liftPID(50); //keep in percent form, not decimal form 
    // }else if(operatorStick.getRawButton(7)){
    //   liftPID(0);
    //}else{
      liftMotor.set(ControlMode.PercentOutput, 0);
      liftMotor.set(ControlMode.PercentOutput, -operatorStick.getRawAxis(3));
    //}


     //////////////////
     //end of pneumatics */
     //////////////////

    /*if(driveStick.getRawButtonPressed(5) && driveStick.getRawButtonPressed(6)){
      brakeStatus = !brakeStatus;
      setBrakeMode();
    }*/

    //lift
    /*
    if(driveStick.getRawButton(1)) {
      liftMotor.set(ControlMode.Position, (int)(25000)); //pid low
    } 
    else if(driveStick.getRawButton(2)) {
      liftMotor.set(ControlMode.Position, (int)(105000));//0.65*maxPosition)); //pid high
    } 
    else{
      liftMotor.set(ControlMode.PercentOutput, 0);
    }*/

    if(driveStick.getRawButtonPressed(9) || driveStick.getRawButtonPressed(10)){
      //set the flags for the amp code
      zeroCompleted = false;
    }

    if (manualMode){
      //arm in out
      /*if(operatorStick.getRawButton(5)){ //go in
        armMotor.set(ControlMode.PercentOutput, -0.4);
      } else if(operatorStick.getRawButton(6)) { //go out
        armMotor.set(ControlMode.PercentOutput, 0.4);
      }
      else if(operatorStick.getRawButton(9)){
        zeroArm(true);
      }
      else if(operatorStick.getRawButton(10)){
        zeroArm(false);
      }
      else if(operatorStick.getRawButtonReleased(5) || operatorStick.getRawButtonReleased(6)){
        //armMotor.set(ControlMode.Position, armMotor.getSelectedSensorPosition());
        armMotor.set(ControlMode.PercentOutput, -0.2);
      }*/

      
      if (operatorStick.getRawButton(6)){
        armGoal = 9700;
      }
      else if (operatorStick.getRawButton(8)){
        armGoal = 14000;
      }
      else if (operatorStick.getRawButton(5)){
        armGoal = 0;
      }
      else{
        armPID(armGoal);
      }

      if (operatorStick.getRawButton(1)){
        armRotator.set(ControlMode.Position, 150000);
      }
      else{
        armRotator.set(ControlMode.PercentOutput, -operatorStick.getRawAxis(1));
      }
      SmartDashboard.putNumber("Arm Rot POS", armRotator.getSelectedSensorPosition());

      

      /*
      //rotate
      if(operatorStick.getRawButton(1)){
        armRotator.set(ControlMode.PercentOutput, 0.4);
      }else if(operatorStick.getRawButton(3)){
        armRotator.set(ControlMode.PercentOutput, -0.2);
      }else{
        armRotator.set(ControlMode.PercentOutput, 0);
      }

      //lift
      if(operatorStick.getRawButton(7)){
        liftMotor.set(ControlMode.PercentOutput, 0.5); //theoretical up
        //liftMotorSlave.set(ControlMode.PercentOutput, 0.3);
      }
      else if(operatorStick.getRawButton(8)){
        liftMotor.set(ControlMode.PercentOutput, -0.5); //theoretial down
         //liftMotorSlave.set(ControlMode.PercentOutput, -0.1);
      }
      else{
        liftMotor.set(ControlMode.PercentOutput, 0.0);
         //liftMotorSlave.set(ControlMode.PercentOutput, 0.0);
      }
      */
    }
    else{ //not in manual mode

      //arm in out
      /*if (operatorStick.getRawButtonPressed(6)){
        armGoal += (armGoalIncrement * armMaxDistance);
        armGoal = armGoal > armMaxDistance ? armMaxDistance : armGoal;
      }
      if(operatorStick.getRawButtonPressed(5)){
        armGoal -= (armGoalIncrement * armMaxDistance);
        armGoal = armGoal < 300 ? 300 : armGoal;
      }
      //arm in out gtting zeros
      if(operatorStick.getRawButton(9)){
        zeroArm(true);
      }
      else if(operatorStick.getRawButton(10)){
        zeroArm(false);
      }
      else{
        armMotor.set(ControlMode.Position, armGoal);
      }*/

      //arm rotation
      if(operatorStick.getRawButton(1)){
      armRotatePID(0); //keep in percent form, not decimal form 
      }else if(operatorStick.getRawButton(4)){
      armRotatePID(100);
      }
      //lift
      double curLiftPosition = liftMotor.getSelectedSensorPosition();
      if(operatorStick.getRawButtonPressed(7)){
        //go up

      }
      else if(operatorStick.getRawButtonPressed(8)){
        //go down (it dont go down!)
      }

    }
  }

  public void armPID(double Goal){
    double error = Goal - armMotor.getSelectedSensorPosition();
    double p = 0.00025;
    double output = error*p;

    double max = 0.4;
    output = output > max ? max: output;
    output = output < -max ? -max: output;

    armMotor.set(ControlMode.PercentOutput, output);
  }

  public void liftPID(double percentGoal){
    double goal = liftMaxPosition * percentGoal/100;
    // double error = goal - liftMotor.getSelectedSensorPosition();
    // double liftP = 0.0001;
    // double output = error * liftP;
    // if(output > maxLiftSpeedUp){
    //   output = maxLiftSpeedUp;
    // }else if(output < maxLiftSpeedDown){
    //   output = maxLiftSpeedDown;
    // }
    liftMotor.set(ControlMode.Position, goal);
  }
  public void armRotatePID(double percentGoal){
    double goal = armMaxPosition * percentGoal/100;
    
    armRotator.set(ControlMode.Position, goal);
  }


  public void zeroArm(boolean findingMin){
    if (Math.abs(armMotor.getStatorCurrent()) > zeroSettingAmperage){
      if (zeroSettingBOOL == false){
        zeroSettingTimeFlag = timer.get() + zeroSettingTime;
      }
      zeroSettingBOOL = true;
    }
    else{
      zeroSettingTimeFlag = Double.MAX_VALUE;
      zeroSettingBOOL = false;
    }
    if (zeroSettingBOOL){
      if (timer.get() > zeroSettingTimeFlag){
        zeroCompleted = true;
        if (findingMin){
          armMotor.setSelectedSensorPosition(0);
        }
        else{
          armMax = armMotor.getSelectedSensorPosition();
        }
      }
    }
    if (!zeroCompleted){
      if (findingMin){
        armMotor.set(ControlMode.PercentOutput,-0.2);
      }
      else{
        armMotor.set(ControlMode.PercentOutput, 0.2);
      }
    }
    else{
      armMotor.set(ControlMode.PercentOutput,0);
    }
  }

  public void setBrakeMode(){
    if (brakeStatus == true){
      leftDrive1.setNeutralMode(NeutralMode.Brake);
      leftDrive2.setNeutralMode(NeutralMode.Brake);
      rightDrive1.setNeutralMode(NeutralMode.Brake);
      rightDrive2.setNeutralMode(NeutralMode.Brake);
    }
    else{
      leftDrive1.setNeutralMode(NeutralMode.Coast);
      leftDrive2.setNeutralMode(NeutralMode.Coast);
      rightDrive1.setNeutralMode(NeutralMode.Coast);
      rightDrive2.setNeutralMode(NeutralMode.Coast);
    }
  }
  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public void balancingDistancePID(double goal, double P, double D){
    double position = (leftDrive1.getSelectedSensorPosition() + rightDrive1.getSelectedSensorPosition()) / 2;
    double error = goal - position;


    if(error <= goal - 1000){
      leftDrive1.set(ControlMode.PercentOutput, 0);
      leftDrive2.set(ControlMode.PercentOutput, 0);
      rightDrive1.set(ControlMode.PercentOutput, 0);
      rightDrive2.set(ControlMode.PercentOutput, 0);
    }
  }

    
   public void balancingPID(double P, double D, double max){
    double error = navX.getRoll();
    double demand = error * P;

    demand = (demand > max)? max:demand;
    demand = (demand < -max)? -max:demand;

    leftDrive1.set(ControlMode.PercentOutput, demand);
    rightDrive1.set(ControlMode.PercentOutput, demand);
  }

  public void drive(){
    double x;
    double y;
    double maxY = 1;

    //Negative X for 2023 practice bot, positive X for 2022 comp bot
    x = -driveStick.getRawAxis(0);
    y = (driveStick.getRawAxis(3) + 1 )/2 - (driveStick.getRawAxis(4) + 1 )/2 ;

    x = x * x *x; 
    if(y > maxY){
      y = maxY;
    }else if(y < -maxY){
      y = -maxY;
    }
    leftDrive1.set(ControlMode.PercentOutput, y + x );
    rightDrive1.set(ControlMode.PercentOutput, y - x);
  }

  public void driveButBetter(){
    double x;
    double y;
    double maxY = 1;
    //max goal = -20000
    double goal = 20000;

    //Negative X for 2023 practice bot, positive X for 2022 comp bot
    x = -driveStick.getRawAxis(0) / 3;
    y = (driveStick.getRawAxis(3) + 1 )/2 - (driveStick.getRawAxis(4) + 1 )/2 ;

    //x = x * x *x; 
    if(y > maxY){
      y = maxY;
    }else if(y < -maxY){
      y = -maxY;
    }

    x = (Math.abs(x) < 0.1)? 0 : x;
    x = (x > 0.1)? ((x-0.1)/0.9) : x;
    x = (x < -0.1)? ((x+0.1)/0.9) : x;

    y = (Math.abs(y) < 0.1)? 0 : y;
    y = (y > 0.1)? ((y-0.1)/0.9) : y;
    y = (y < -0.1)? ((y+0.1)/0.9) : y;

    if(x == 0 && y == 0) {
      leftDrive1.set(ControlMode.Disabled, 0);
      rightDrive1.set(ControlMode.Disabled, 0);
    } else {
      leftDrive1.set(ControlMode.Velocity, (y + x)*goal );
      rightDrive1.set(ControlMode.Velocity, (y - x)*goal);
    }

    SmartDashboard.putNumber("LeftDriveTargetVelocity", (y + x)*goal);
    SmartDashboard.putNumber("RightDriveTargetVelocity", (y - x)*goal);

    
  }
  public void shooter(){
    if(timer.get() <= shooterFlag){
      armWheels.set(ControlMode.PercentOutput, -power); 
    }else{
      armWheels.set(ControlMode.PercentOutput, 0);
    }
  }
}