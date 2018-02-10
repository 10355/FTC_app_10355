package org.firstinspires.ftc.teamcode.HardwareProfiles;

import android.text.method.Touch;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;


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
    public DcMotor motorLift = null;            //Declare the motor
    public DcMotor motorRelicArm = null;        //Declare the motor
    public DcMotor motorLinearSlide = null;     //Declare the motor
    public OpticalDistanceSensor ods;           //Declare the sensor
    public ColorSensor colorSensorRight;        //Declare the Color Sensor
    public ColorSensor colorSensorLeft;         //Declare the Color Sensor
    public TouchSensor touchSensor;             //Declare the Touch Sensor
    public GyroSensor sensorGyro;               //Declare the GyroNew sensor
    public ModernRoboticsI2cGyro mrGyro;        //Declare the MR GyroNew
    public Servo servoRight;                    //Declare the servo
    public Servo servoLeft;                     //Declare the servo
    public Servo servoLiftRight;                //Declare the servo
    public Servo servoLiftLeft;                 //Declare the servo
    public Servo servoRelicGrab;                //Declare the servo
    public Servo servoStone;                    //Declare the servo
    public Servo servoBlockExit;                //Declare the block exit servo
//    public TouchSensor limitUp;             //Declare the Lift up limit switch
//    public TouchSensor limitDown;           //Declare the Lift down limit switch
    public BNO055IMU imu = null;
    public ModernRoboticsI2cRangeSensor rangeSensor;
    public ModernRoboticsI2cRangeSensor rangeSensorRight;

    public DeviceInterfaceModule dim;                  // Device Object
    public DigitalChannel        limitUp;                // Device Object
    public DigitalChannel        limitDown;            // Device Object


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
            I2cAddr i2CAddressColorRight = I2cAddr.create8bit(0x4c);
            I2cAddr i2CAddressColorLeft = I2cAddr.create8bit(0x3c);
            colorSensorRight = hwMap.colorSensor.get("colorR"); //Map the sensor to the hardware
            colorSensorLeft = hwMap.colorSensor.get("colorL"); //Map the sensor to the hardware
            colorSensorRight.setI2cAddress(i2CAddressColorRight);
            colorSensorLeft.setI2cAddress(i2CAddressColorLeft);
            colorSensorRight.enableLed(true);
            colorSensorLeft.enableLed(true);

            //Define the range sensor
            I2cAddr i2CAddressRangeLeft = I2cAddr.create8bit(0x28);
            rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");
            rangeSensor.setI2cAddress(i2CAddressRangeLeft);
            I2cAddr i2CAddressRangeRight = I2cAddr.create8bit(0x2a);
            rangeSensorRight = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeRight");
            rangeSensorRight.setI2cAddress(i2CAddressRangeRight);

            //Define the the limit switches for the glyph box
            limitUp= hwMap.get(DigitalChannel.class, "liftUp");
            limitDown= hwMap.get(DigitalChannel.class, "liftDown");
            limitUp.setMode(DigitalChannel.Mode.INPUT);          // Set the direction of each channel
            limitDown.setMode(DigitalChannel.Mode.INPUT);

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

            motorLift = hwMap.dcMotor.get("blockHolder");
            motorLift.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorLift.setPower(0);
            motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorRelicArm = hwMap.dcMotor.get("relicArm");
            motorRelicArm.setDirection(DcMotor.Direction.FORWARD);
            motorRelicArm.setPower(0);
            motorRelicArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRelicArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorLinearSlide = hwMap.dcMotor.get("linearSlide");
            motorLinearSlide.setDirection(DcMotor.Direction.FORWARD);
            motorLinearSlide.setPower(0);
            motorLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //Setup the servos
            servoRight = hwMap.servo.get("servo0");         // right gem arm
            servoLeft = hwMap.servo.get("servo1");          // left gem arm
            servoLiftRight = hwMap.servo.get("liftR");      // glyph grabber right
            servoLiftLeft = hwMap.servo.get("liftL");       // glyph grabber left
            servoRelicGrab = hwMap.servo.get("relicGrab");  // relic grabbing servo
            servoStone = hwMap.servo.get("servoStone");     // stone placement servo
            servoBlockExit = hwMap.servo.get("blockExit");  // glyph ejector servo
        }
    }

}