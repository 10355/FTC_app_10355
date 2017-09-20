/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
    Team:       10355 - Project Peacock
    Autonomous Program - Red Strategy #1
    Alliance Color: Red
    Robot Starting Position: Red zone, wall near ramp
    Strategy Description:
        - Press correct button on beacon near ramp
        - Press correct button on beacon furthest from ramp
        - Park a wheel on the red ramp

    Hardware Setup:
        - 4 mecanum wheels with encoder on LF wheel - encoder utilized for measuring distance for fwd/rev drive operation
        - Arm Motor with encoder - controls mechanism for dumping particles into ramp
        - Gyro sensor located at the center of the robot - utilized to compensate for drift
        - 1 x Color sensor (colorSensorLeft)- utilized to identify beacon color
        - 1 x Touch sensor - utilized to identify when robot touches wall with the front of the robot
        - 1 x Optical Distance Sensor (ODS) - utilized to locate the white lines on the floor
        - 1 x Motorola Camera - Utilized for Vuforia positioning of the robot on the field

    State Order:
        - ACQUIRE_BLUE_BEACON_RIGHT       // moves from the wall to the first beacon closest to the ramp
        - PUSH_BUTTON_RED               // Identifies which button to press, right or left
        - PRESS_BUTTON                  // presses the correct button - also validates that the button is pressed
                                        // and attempts to press over again until the button is pressed
        - ACQUIRE_BLUE_BEACON_LEFT      // moves from the wall to the second beacon on the right of the field
        - PUSH_BUTTON_RED               // Identifies which button to press, right or left
        - PRESS_BUTTON                  // presses the correct button - also validates that the button is pressed
                                        // and attempts to press over again until the button is pressed
        - END_GAME                      // identifies the last actions before the end of autonomous mode
        - RAMP                          // For this strategy, the robot will end with a wheel parked on the ramp
        - HALT                          // Shutdown sequence for autonomous mode

 */
package org.firstinspires.ftc.teamcode.Opmodes;

/**
 * Import the classes we need to have local access to.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;
import org.firstinspires.ftc.teamcode.Libs.Shooter;
import org.firstinspires.ftc.teamcode.Libs.VuforiaLib;
import java.util.List;
import java.util.Locale;

/**
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "Mecanum Gyro", group = "COMP")

public class MecanumGyro extends LinearOpMode {

    /**
     * Instantiate all objects needed in this class
     */

    private final static HardwareTestPlatform robot = new HardwareTestPlatform();
    private VuforiaLib myVuforia = new VuforiaLib();
    private ElapsedTime runtime = new ElapsedTime();
    private LinearOpMode opMode = this;
    private DataLogger Dl;                          //Datalogger object
    private String alliance = "blue";                //Your current alliance
    private State state = State.DRIVE;    //Machine State
    /**
     * Define global variables
     */

    private double colorRight = 0;
    private double colorLeft = 0;
    private double colorLeftBlue = 0;
    private double colorLeftRed = 0;
    private double headingAngle = 0;
    double radians = 0;
    public double LF, RF, LR, RR;
    public double currentZint = 0;
    private long z = 0;
    public double zCorrection = 0;
    public double initZ = 0;
    private double changeSpeed = 0;
    private double motorCorrectCoefficient = .05;
    public double myCurrentMotorPosition;
    private boolean tel = false;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;

    public void runOpMode() {
        /**
         * Setup the init state of the robot.  This configures all the hardware that is defined in
         * the HardwareTestPlatform class.
         */
        robot.init(hardwareMap);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        /**
         *  Create the DataLogger object.
         */
        createDl();

        /**
         * Initialize Vuforia and retrieve the list of trackable objects.
         */
        robot.sensorGyro.calibrate();
        while (robot.sensorGyro.isCalibrating()) {
            telemetry.addData("Waiting on Gyro Calibration", "");
            telemetry.update();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(1000);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        /**
         * Deploy the color sensor
         */

        robot.servoLeft.setPosition(.5);
        robot.servoRight.setPosition(1);
        sleep(2000);

        /**
         * Start the opMode
         */
        waitForStart();

        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        // Start the logging of measured acceleration


        while (opModeIsActive()) {
            /**
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             * This is the section of code you should change for your robot.
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             */

            switch (state) {
                case TRANSLATE:
                    translateTimeMR(20, .5, 0);
                    state = State.HALT;
                    break;
                case MECANUM:
                    correctMecDrive(4, .75, 90);
                    state = State.HALT;
                    break;
                case AUTONOMUS_BLUE_FRONT:
                    if (alliance == "blue") {
                        colorLeftBlue = robot.colorSensorLeft.blue();
                        colorLeftRed = robot.colorSensorLeft.red();

                        if (colorLeftBlue > colorLeftRed) {  //Color Sensor sees the red ball
                            correctMecDrive(1, 0.25, 0);  //Drive backward and knock off the ball
                        }
                        else {
                            correctMecDrive(1, 0.25, 180);  //Drive forward and knock off the ball
                        }
                        robot.servoLeft.setPosition(.5);
                        correctMecDrive(4, .75, 90);
                    }

                    state = State.HALT;
                    break;
                case RANGE:
                    while (opModeIsActive()) {
                        telemetry.addData("raw ultrasonic", robot.rangeSensor.rawUltrasonic());
                        telemetry.addData("raw optical", robot.rangeSensor.rawOptical());
                        telemetry.addData("cm optical", "%.2f cm", robot.rangeSensor.cmOptical());
                        telemetry.addData("cm", "%.2f cm", robot.rangeSensor.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }
                    break;
                case COLOR_SENSOR:
                    //colorRight = robot.colorSensorRight.argb();
                    //colorLeft = robot.colorSensorLeft.argb();
                    //telemetry.addData("Right RGB", String.valueOf(colorRight));
                    //telemetry.addData("Right Blue", String.valueOf(robot.colorSensorRight.blue()));
                    //telemetry.addData("Right Red", String.valueOf(robot.colorSensorRight.red()));
                    //telemetry.addData("Left RGB", String.valueOf(colorLeft));
                    //telemetry.addData("Left Blue", String.valueOf(robot.colorSensorLeft.blue()));
                    //telemetry.addData("Left Red", String.valueOf(robot.colorSensorLeft.red()));
                    //telemetry.update();

                    break;
                case TAIL:
                    robot.servoRight.setPosition(1);
                    robot.servoLeft.setPosition(0);

                    sleep(2000);
                    robot.servoRight.setPosition(.5);
                    robot.servoLeft.setPosition(.5);
                    sleep(2000);
                    //robot.servoElbow.setPosition(1);
                    break;
                case DRIVE:
                    double runTime = 5;
                    double targetTime;
                    targetTime = opMode.getRuntime() + runTime;

                    while (targetTime > opMode.getRuntime()) {
                        double headingAngle;
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        headingAngle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

                        telemetry.addData("heading", String.valueOf(headingAngle));
                        telemetry.addData("TargetTime", String.valueOf(targetTime));
                        telemetry.addData("RunTime", String.valueOf(opMode.getRuntime()));
                        telemetry.update();

                        translateTimeMR(3.6, 0.5, 0);

                        pivotLeft(0.25, 85);

                        translateRange(.5, 0, 25);

                        translateRange(.5, -90, 15);

                        wait(1);

                        translateTimeMR(0.5, 0.5, 90);

                        wait(1);

                        translateRange(.5, -90, 15);



                        motorsHalt();

                        opMode.idle();
                    }
                    state = State.HALT;

                    break;
                case HALT:
                    robot.motorLF.setPower(0);
                    robot.motorLR.setPower(0);
                    robot.motorRF.setPower(0);
                    robot.motorRR.setPower(0);
                    //Stop the DataLogger
                    dlStop();

                    //Exit the OpMode
                    requestOpModeStop();
                    break;
            }
        }
    }

    public void wait(int time) {
        double then = 0;

        then = opMode.getRuntime() + time;

        while (opMode.getRuntime() < then) {

        }
    }

    /**
     * Pivot the robot to a new heading. 0 is straight ahead, 1 to 179 is to the left -1 to -179 is
     * to the right.
     */
    public void pivotLeft(double power, double heading) {

        angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initZ = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        //initZ = robot.mrGyro.getIntegratedZValue();
        currentZint = initZ;

        while (currentZint < heading && opMode.opModeIsActive()) {
            /**
             * We are pivoting left so reverse power to the left motor
             */
            LR = -power;
            LF = -power;
            RR = power;
            RF = power;

            setPower(LF, LR, RF, RR);

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (tel) {
                telemetry();
                logData();
            }
            opMode.idle();
        }
        motorsHalt();
    }


    /**
     * Translate on a heading for a defined period of time.
     */
    public void translateTimeMR(double timeOut, double power, double heading) {
        double timeOutTime;
        initZ = robot.mrGyro.getIntegratedZValue();
        currentZint = robot.mrGyro.getIntegratedZValue();
        radians = getRadians(heading);

        timeOutTime = runtime.time() + timeOut;

        while (opMode.opModeIsActive() && runtime.time() < timeOutTime) {
            LF = calcLF(radians, power);
            RF = calcRF(radians, power);
            LR = calcLR(radians, power);
            RR = calcRR(radians, power);

            currentZint = robot.mrGyro.getIntegratedZValue();

            z = Math.round(currentZint);
            telemetry.addData("Z", String.valueOf(z));

            telemetry();

            if (currentZint != initZ) {  //Robot has drifted off course
                zCorrection = Math.abs(initZ - currentZint);

                courseCorrect();
            } else {
                zCorrection = 0;
            }

            setPower(LF, LR, RF, RR);

            myCurrentMotorPosition = robot.motorLF.getCurrentPosition();

            if (tel) {
                telemetry();
                logData();
            }

            opMode.idle();
        }

        motorsHalt();

    }

    public void translateMR(double power, double heading) {

        initZ = robot.mrGyro.getIntegratedZValue();
        currentZint = robot.mrGyro.getIntegratedZValue();
        radians = getRadians(heading);



        LF = calcLF(radians, power);
        RF = calcRF(radians, power);
        LR = calcLR(radians, power);
        RR = calcRR(radians, power);

        currentZint = robot.mrGyro.getIntegratedZValue();

        z = Math.round(currentZint);
        telemetry.addData("Z", String.valueOf(z));

        telemetry();

        if (currentZint != initZ) {  //Robot has drifted off course
            zCorrection = Math.abs(initZ - currentZint);

            courseCorrect();
        } else {
            zCorrection = 0;
        }

        setPower(LF, LR, RF, RR);

        myCurrentMotorPosition = robot.motorLF.getCurrentPosition();

        if (tel) {
            telemetry();
            logData();
        }

        opMode.idle();


    }

    /**
     * Translate on a heading for a defined period of time.
     */
    public void translateTime(double timeOut, double power, double heading) {
        double timeOutTime;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initZ = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        radians = getRadians(heading);

        timeOutTime = runtime.time() + timeOut;

        while (opMode.opModeIsActive() && runtime.time() < timeOutTime) {
            LF = calcLF(radians, power);
            RF = calcRF(radians, power);
            LR = calcLR(radians, power);
            RR = calcRR(radians, power);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentZint = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

            z = Math.round(currentZint);
            telemetry.addData("Z", String.valueOf(z));

            telemetry();

            if (currentZint != initZ) {  //Robot has drifted off course
                zCorrection = Math.abs(initZ - currentZint);

                courseCorrect();
            } else {
                zCorrection = 0;
            }

            setPower(LF, LR, RF, RR);

            myCurrentMotorPosition = robot.motorLF.getCurrentPosition();

            if (tel) {
                telemetry();
                logData();
            }

            opMode.idle();
        }

        motorsHalt();

    }

    /**
     * Translate on a heading for a defined period of time.
     */
    public void translateRange(double power, double heading, int range) {
        double timeOutTime;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initZ = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        radians = getRadians(heading);

        while (opMode.opModeIsActive() &&
                robot.rangeSensor.rawUltrasonic() > range) {
            LF = calcLF(radians, power);
            RF = calcRF(radians, power);
            LR = calcLR(radians, power);
            RR = calcRR(radians, power);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentZint = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

            z = Math.round(currentZint);
            telemetry.addData("Z", String.valueOf(z));

            telemetry();

            if (currentZint != initZ) {  //Robot has drifted off course
                zCorrection = Math.abs(initZ - currentZint);

                courseCorrect();
            } else {
                zCorrection = 0;
            }

            setPower(LF, LR, RF, RR);

            myCurrentMotorPosition = robot.motorLF.getCurrentPosition();

            if (tel) {
                telemetry();
                logData();
            }

            opMode.idle();
        }

        motorsHalt();

    }
    private void correctDrive(double time, double power) {

        double timeout = getRuntime() + time;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double intz = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

        while(getRuntime() < timeout) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            headingAngle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

            telemetry.addData("Gyro", String.valueOf(headingAngle));
            telemetry.update();

            if (headingAngle == intz) {
                robot.motorLF.setPower(power);
                robot.motorRR.setPower(power);
                robot.motorLR.setPower(power);
                robot.motorRF.setPower(power);
            }

            if (headingAngle < intz) {      //Robot has turned right
                robot.motorLF.setPower(power*0.9);
                robot.motorRR.setPower(power);
                robot.motorLR.setPower(power*0.9);
                robot.motorRF.setPower(power);
            }

            if (headingAngle > intz) {      //Robot has turned left
                robot.motorLF.setPower(power);
                robot.motorRR.setPower(power*0.9);
                robot.motorLR.setPower(power);
                robot.motorRF.setPower(power*0.9);
            }

        }
    }

    private void correctMecDrive(double time, double power, double heading) {
        double timeout = getRuntime() + time;
        double radians = getRadians(heading);


        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double intz = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

        double v1 = power * Math.sin(radians + (Math.PI / 4));
        double v2 = power * Math.sin(radians + (Math.PI / 4));
        double v3 = power * Math.cos(radians + (3*(Math.PI / 4)));
        double v4 = power * Math.cos(radians + (3*(Math.PI / 4)));

        while(getRuntime() < timeout) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            headingAngle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

            telemetry.addData("Gyro", String.valueOf(headingAngle));
            telemetry.update();

            if (headingAngle == intz) {
                robot.motorLF.setPower(v1);
                robot.motorRR.setPower(v2);
                robot.motorLR.setPower(v3);
                robot.motorRF.setPower(v4);
            }

            if (headingAngle < intz) {      //Robot has turned right
                robot.motorLF.setPower(v1-(0.02*Math.abs(headingAngle)));
                robot.motorRR.setPower(v2);
                robot.motorLR.setPower(v3-(0.02*Math.abs(headingAngle)));
                robot.motorRF.setPower(v4);
            }

            if (headingAngle > intz) {      //Robot has turned left
                robot.motorLF.setPower(v1);
                robot.motorRR.setPower(v2-(0.02*Math.abs(headingAngle)));
                robot.motorLR.setPower(v3);
                robot.motorRF.setPower(v4-(0.02*Math.abs(headingAngle)));
            }
        }
    }
    public void courseCorrect() {
        if (currentZint > initZ) {  //Robot has drifted left
            telemetry.addData("Course", "LEFT");
            LF = LF + (motorCorrectCoefficient);
            LR = LR + (motorCorrectCoefficient);
            RF = RF - (motorCorrectCoefficient);
            RR = RR - (motorCorrectCoefficient);
        }

        if (currentZint < initZ) {  //Robot has drifted right
            telemetry.addData("Course", "RIGHT");
            LF = LF - (motorCorrectCoefficient);
            LR = LR - (motorCorrectCoefficient);
            RF = RF + (motorCorrectCoefficient);
            RR = RR + (motorCorrectCoefficient);
        }
    }

    public void motorsHalt() {
        robot.motorLF.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
    }

    /**
     * Set the power level of the motors.
     */
    public void setPower(double LF, double LR, double RF, double RR) {
        robot.motorLF.setPower(LF);
        robot.motorRF.setPower(RF);
        robot.motorLR.setPower(LR);
        robot.motorRR.setPower(RR);
    }

    /**
     * Calculate the wheel speeds.
     *
     * @return wheel speed
     */
    public double calcLF(double radians, double power) {
        LF = power * Math.sin(radians + (Math.PI / 4)) + changeSpeed;

        if (LF > 1 || LF < -1) {
            LF = 0;
        }

        return LF;
    }

    public double calcRF(double radians, double power) {
        RF = power * Math.cos(radians + (Math.PI / 4)) - changeSpeed;

        if (RF > 1 || RF < -1) {
            RF = 0;
        }

        return RF;
    }

    public double calcLR(double radians, double power) {
        LR = power * Math.cos(radians + (Math.PI / 4)) + changeSpeed;

        if (LR > 1 || LR < -1) {
            LR = 0;
        }

        return LR;
    }

    public double calcRR(double radians, double power) {
        RR = power * Math.sin(radians + (Math.PI / 4)) - changeSpeed;

        if (RR > 1 || RR < -1) {
            RR = 0;
        }

        return RR;
    }
    public double getRadians(double heading) {
        radians = heading * (Math.PI / 180);

        return radians;
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /**
     * Setup the dataLogger
     * The dataLogger takes a set of fields defined here and sets up the file on the Android device
     * to save them to.  We then log data later throughout the class.
     */
    private void createDl() {

        Dl = new DataLogger("AutoMecanumSimpleTest" + runtime.time());
        Dl.addField("runTime");
        Dl.addField("Alliance");
        Dl.newLine();
    }

    /**
     * Log data to the file on the phone.
     */
    private void logData() {

        Dl.addField(String.valueOf(runtime.time()));
        Dl.addField(String.valueOf(alliance));
        Dl.newLine();
    }

    /**
     * Transmit telemetry.
     */
    private void telemetry() {

        opMode.telemetry.addData("Current Z Int", String.valueOf(currentZint));
        opMode.telemetry.addData("Z Correction", String.valueOf(zCorrection));
        opMode.telemetry.addData("LF Encoder", String.valueOf(robot.motorLF.getCurrentPosition()));
        opMode.telemetry.addData("LR Encoder", String.valueOf(robot.motorLR.getCurrentPosition()));
        opMode.telemetry.addData("RF Encoder", String.valueOf(robot.motorRF.getCurrentPosition()));
        opMode.telemetry.addData("RR Encoder", String.valueOf(robot.motorRR.getCurrentPosition()));
        opMode.telemetry.addData("LF", String.valueOf(LF));
        opMode.telemetry.addData("LR", String.valueOf(LR));
        opMode.telemetry.addData("RF", String.valueOf(RF));
        opMode.telemetry.addData("RR", String.valueOf(RR));
        opMode.telemetry.addData("raw ultrasonic", robot.rangeSensor.rawUltrasonic());
        opMode.telemetry.addData("raw optical", robot.rangeSensor.rawOptical());
        opMode.telemetry.addData("cm optical", "%.2f cm", robot.rangeSensor.cmOptical());
        opMode.telemetry.addData("cm", "%.2f cm", robot.rangeSensor.getDistance(DistanceUnit.CM));
        //opMode.telemetry.addData("Right RGB", String.valueOf(colorRight));
        //opMode.telemetry.addData("Right Blue", String.valueOf(robot.colorSensorRight.blue()));
        //opMode.telemetry.addData("Right Red", String.valueOf(robot.colorSensorRight.red()));
        //opMode.telemetry.addData("Left RGB", String.valueOf(colorLeft));
        //opMode.telemetry.addData("Left Blue", String.valueOf(robot.colorSensorLeft.blue()));
        //opMode.telemetry.addData("Left Red", String.valueOf(robot.colorSensorLeft.red()));
        opMode.telemetry.addData("LF", String.valueOf(LF));
        opMode.telemetry.addData("RF", String.valueOf(RF));
        opMode.telemetry.addData("LR", String.valueOf(LR));
        opMode.telemetry.addData("RR", String.valueOf(RR));
        opMode.telemetry.update();
    }

    /**
     * Stop the DataLogger
     */
    private void dlStop() {
        Dl.closeDataLogger();

    }

    /**
     * Enumerate the States of the machine.
     */
    enum State {
        HALT, DRIVE, TAIL, COLOR_SENSOR, RANGE, AUTONOMUS_BLUE_FRONT, MECANUM, TRANSLATE
    }

}
