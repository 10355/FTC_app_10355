package org.firstinspires.ftc.teamcode.HardwareProfiles;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


/**
 * This is the hardware definition for the Hardware test/reference platform.  This class should be
 * instantiated in your opmode to allow access to the hardware.
 * <p>
 * Example:
 * <p>
 * private HardwareTestPlatform robot = new HardwareTestPlatform();
 */

public class HardwareTestPlatform {
    /* Public OpMode members. */
    public DcMotor motorLF = null;              //Declare the motor
    public DcMotor motorRF = null;              //Declare the motor
    public DcMotor motorLR = null;              //Declare the motor
    public DcMotor motorRR = null;              //Declare the motor
    public DcMotor motorLift = null;          //Declare the motor
    public OpticalDistanceSensor ods;           //Declare the sensor
    public ColorSensor colorSensorRight;        //Declare the Color Sensor
    public ColorSensor colorSensorLeft;        //Declare the Color Sensor
    public TouchSensor touchSensor;             //Declare the Touch Sensor
    public GyroSensor sensorGyro;               //Declare the GyroNew sensor
    public ModernRoboticsI2cGyro mrGyro;        //Declare the MR GyroNew
    public Servo servoRight;                   //Declare the servo
    public Servo servoLeft;                   //Declare the servo
    public Servo servoLiftRight;                   //Declare the servo
    public Servo servoLiftLeft;                   //Declare the servo
    public BNO055IMU imu = null;
    public ModernRoboticsI2cRangeSensor rangeSensor;

    //Wheel Setup
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // AndyMark 40
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_MM         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)) * 25.4;

    /* Constructor */
    public HardwareTestPlatform() {

    }

    /**
     * Map all the robots hardware
     *
     * @param ahwMap Input hardwaremap
     */
    public void init(HardwareMap ahwMap) {
        String platform = "revPrototype";
        // Save reference to Hardware map
        HardwareMap hwMap;
        hwMap = ahwMap;

        if (platform.equals("revPrototype")) {
            //Define the gyro
            sensorGyro = hwMap.gyroSensor.get("gyro");     //Point to the gyro in the configuration file
            mrGyro = (ModernRoboticsI2cGyro) sensorGyro;         //MR GyroNew

            //Define the color sensors
            //I2cAddr i2CAddressColorRight = I2cAddr.create8bit(0x4c);
            I2cAddr i2CAddressColorLeft = I2cAddr.create8bit(0x3c);
            //colorSensorRight = hwMap.colorSensor.get("colorR"); //Map the sensor to the hardware
            colorSensorLeft = hwMap.colorSensor.get("colorL"); //Map the sensor to the hardware
            //colorSensorRight.setI2cAddress(i2CAddressColorRight);
            colorSensorLeft.setI2cAddress(i2CAddressColorLeft);
            //colorSensorRight.enableLed(true);
            colorSensorLeft.enableLed(true);

            //Define the range sensor
            rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");


            //Setup the drive motors
            motorLF = hwMap.dcMotor.get("lf");
            motorLF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            motorLF.setPower(0);
            motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorLR = hwMap.dcMotor.get("lr");
            motorLR.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            motorLR.setPower(0);
            motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorRF = hwMap.dcMotor.get("rf");
            motorRF.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorRF.setPower(0);
            motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorRR = hwMap.dcMotor.get("rr");
            motorRR.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorRR.setPower(0);
            motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorLift = hwMap.dcMotor.get("lift");
            motorLift.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorLift.setPower(0);
            motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //motorLift = hwMap.dcMotor.get("lift");
            //motorLift.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            //motorLift.setPower(0);
            //motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //Setup the servos
            servoRight = hwMap.servo.get("servo0");
            servoLeft = hwMap.servo.get("servo1");
            servoLiftRight = hwMap.servo.get("liftR");
            servoLiftLeft = hwMap.servo.get("liftL");

        }
    }
}