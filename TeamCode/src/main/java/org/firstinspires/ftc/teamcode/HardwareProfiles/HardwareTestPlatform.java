package org.firstinspires.ftc.teamcode.HardwareProfiles;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is the hardware definition for the Hardware test/reference platform.  This class should be
 * instantiated in your opmode to allow access to the hardware.
 * <p>
 * Example:
 * <p>
 * private HardwareTestPlatform robot = new HardwareTestPlatform();
 */

public class HardwareTestPlatform {
    //Wheel Setup
    public final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    public final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    public final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            ((WHEEL_DIAMETER_INCHES * 25.41) * Math.PI);

    /* Public OpMode members. */
    public DcMotor motorLF = null;              //Declare the motor
    public DcMotor motorRF = null;              //Declare the motor
    public DcMotor motorLR = null;              //Declare the motor
    public DcMotor motorRR = null;              //Declare the motor
    public DcMotor motorFeeder = null;          //Declare the motor
    public DcMotor motorShooter = null;          //Declare the motor
    public OpticalDistanceSensor ods;           //Declare the sensor
    public ColorSensor colorSensorRight;        //Declare the Color Sensor
    public ColorSensor colorSensorLeft;        //Declare the Color Sensor
    public TouchSensor touchSensor;             //Declare the Touch Sensor
    public GyroSensor sensorGyro;               //Declare the GyroNew sensor
    public ModernRoboticsI2cGyro mrGyro;        //Declare the MR GyroNew
    public Servo servoRight;                   //Declare the servo
    public Servo servoLeft;                   //Declare the servo
    public BNO055IMU imu = null;
    public ModernRoboticsI2cRangeSensor rangeSensor;
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

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
            sensorGyro = hwMap.gyroSensor.get("gyro");     //Point to the gyro in the configuration file
            mrGyro = (ModernRoboticsI2cGyro) sensorGyro;         //MR GyroNew

            //I2cAddr i2CAddressColorRight = I2cAddr.create8bit(0x3c);
            //I2cAddr i2CAddressColorLeft = I2cAddr.create8bit(0x4c);
            //colorSensorRight = hwMap.colorSensor.get("colorR"); //Map the sensor to the hardware
            //colorSensorLeft = hwMap.colorSensor.get("colorL"); //Map the sensor to the hardware
            //colorSensorRight.setI2cAddress(i2CAddressColorRight);
            //colorSensorLeft.setI2cAddress(i2CAddressColorLeft);
            //colorSensorRight.enableLed(true);
            //colorSensorLeft.enableLed(true);
            rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");

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

            servoRight = hwMap.servo.get("servo0");
            servoLeft = hwMap.servo.get("servo1");
        }
        if (platform.equals("mecanum")) {
            //Define the sensors
            ods = hwMap.opticalDistanceSensor.get("ODS");  //Map the sensor to the hardware
            I2cAddr i2CAddressColorRight = I2cAddr.create8bit(0x3c);
            I2cAddr i2CAddressColorLeft = I2cAddr.create8bit(0x4c);

            sensorGyro = hwMap.gyroSensor.get("gyro");     //Point to the gyro in the configuration file
            mrGyro = (ModernRoboticsI2cGyro) sensorGyro;         //MR GyroNew

            touchSensor = hwMap.touchSensor.get("ts");

            // Define and Initialize Motors
            motorLF = hwMap.dcMotor.get("lf");
            motorRF = hwMap.dcMotor.get("rf");
            motorLR = hwMap.dcMotor.get("lr");
            motorRR = hwMap.dcMotor.get("rr");
            motorFeeder = hwMap.dcMotor.get("feeder");
            motorShooter = hwMap.dcMotor.get("shooter");


            motorLF.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorRF.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            motorLR.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorRR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            motorFeeder.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            motorShooter.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors

            // Set all motors to zero power
            motorLF.setPower(0);
            motorRF.setPower(0);
            motorLR.setPower(0);
            motorRR.setPower(0);
            motorFeeder.setPower(0);
            motorShooter.setPower(0);

            motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

        if (platform.equals("buster")) {
            //Define the sensors
            ods = hwMap.opticalDistanceSensor.get("ODS");  //Map the sensor to the hardware
            //I2cAddr i2CAddressColorRight = I2cAddr.create8bit(0x3c);
            //I2cAddr i2CAddressColorLeft = I2cAddr.create8bit(0x4c);

            //colorSensorRight = hwMap.colorSensor.get("colorR"); //Map the sensor to the hardware
            //colorSensorLeft = hwMap.colorSensor.get("colorL"); //Map the sensor to the hardware
            //colorSensorRight.setI2cAddress(i2CAddressColorRight);
            //colorSensorLeft.setI2cAddress(i2CAddressColorLeft);
            //colorSensorRight.enableLed(false);
            //colorSensorLeft.enableLed(false);

            sensorGyro = hwMap.gyroSensor.get("gyro");     //Point to the gyro in the configuration file
            mrGyro = (ModernRoboticsI2cGyro) sensorGyro;         //MR GyroNew

            touchSensor = hwMap.touchSensor.get("ts");


            // Define and Initialize Motors
            motorLF = hwMap.dcMotor.get("lf");
            motorRF = hwMap.dcMotor.get("rf");
            motorLR = hwMap.dcMotor.get("lr");
            motorRR = hwMap.dcMotor.get("rr");
            //motorFeeder = hwMap.dcMotor.get("feeder");


            motorLF.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorRF.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            motorLR.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorRR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            //motorFeeder.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors

            // Set all motors to zero power
            motorLF.setPower(0);
            motorRF.setPower(0);
            motorLR.setPower(0);
            motorRR.setPower(0);
            //motorFeeder.setPower(0);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.

            motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        if (platform.equals("comp")) {
            //Define the sensors
            ods = hwMap.opticalDistanceSensor.get("ODS");  //Map the sensor to the hardware
            touchSensor = hwMap.touchSensor.get("ts");     //Map the sensor to the hardware
            sensorGyro = hwMap.gyroSensor.get("gyro");     //Point to the gyro in the configuration file
            mrGyro = (ModernRoboticsI2cGyro) sensorGyro;         //MR GyroNew


            // Define and Initialize Motors
            motorLR = hwMap.dcMotor.get("lr");
            motorRR = hwMap.dcMotor.get("rr");
            motorFeeder = hwMap.dcMotor.get("feeder");

            motorLR.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorRR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

            // Set all motors to zero power
            motorLR.setPower(0);
            motorRR.setPower(0);

            //Define the sensors
            ods = hwMap.opticalDistanceSensor.get("ODS");  //Map the sensor to the hardware
            //colorSensor = hwMap.colorSensor.get("color1"); //Map the sensor to the hardware
            touchSensor = hwMap.touchSensor.get("ts");     //Map the sensor to the hardware
            sensorGyro = hwMap.gyroSensor.get("gyro");     //Point to the gyro in the configuration file
            mrGyro = (ModernRoboticsI2cGyro) sensorGyro;         //MR GyroNew

            // Define and Initialize Motors
            motorLR = hwMap.dcMotor.get("lr");
            motorRR = hwMap.dcMotor.get("rr");
            motorLR.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorRR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

            // Set all motors to zero power

            motorLR.setPower(0);
            motorRR.setPower(0);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.

            motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}