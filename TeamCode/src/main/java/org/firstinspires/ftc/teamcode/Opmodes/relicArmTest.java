package org.firstinspires.ftc.teamcode.Opmodes;

/*
    Program:    redBack.java
    Opmode Name: Auto RED Back
    Team:       10355 - Project Peacock
    Season:     2017-2018 => Relic Recovery
    Autonomous Program - Red Back Balancing Stone
    Alliance Color: Red
    Robot Starting Position: Red balancing stone farthest from the relic mats
    Strategy Description:
        - Read encrypto picture
        - remove blue gem
        - Place glyph in the correct column
        - Park in the safe zone position

    Hardware Setup:
        - 4 mecanum wheels with encoder on LF wheel - encoder utilized for measuring distance for fwd/rev drive operation
        - Arm Motor with encoder - controls mechanism for retrieving and placing glyphs
        - Arm motor with encoder - controls mechanism for retrieving and placing relics
        - Servos
            - 2 for controlling arms for removing gems
            - 2 for controlling glyph retrieval
            - 2 micro servos for controlling glyph manipulation wheels
            - 1 for controlling relic retrieval system
            - 1 for controlling balancing stone manipulation
        - Gyro sensor located at the center of the robot - utilized to compensate for drift
        - 2 x Color sensor (colorSensorLeft)- utilized to identify gem color
        - 1 x Range Sensor - utilized to position distance from wall during autonomous mode
        - 1 x Motorola Camera - Utilized for decrypting the location of the glyph in autonomous mode

    State Order:
        - VUMark                   // Reads image to determine which column to place the glyph
        - BALL                     // Determines which gem to remove and knocks it from the mount
        - CHECKVU                  // Check VUMark to determine which column to place the glyph in
        - LEFT                     // If Glyph goes in the left column
        - CENTER                   // If Glyph goes in the center column
        - RIGHT                    // If Glyph goes in the center column
        - HALT                     // Shutdown sequence for autonomous mode
        - TEST                     // Temporary state for testing specific functions without
                                   // impacting the rest of the program

 */

/**
 * Import the classes we need to have local access to.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

/**
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "Relic Arm Test", group = "COMP")
@Disabled

public class relicArmTest extends LinearOpMode {

    /**
     * Instantiate all objects needed in this class
     */

    private final static HardwareTestPlatform robot = new HardwareTestPlatform();
    private ElapsedTime runtime = new ElapsedTime();        //ElapsedTime
    private LinearOpMode opMode = this;                     //Opmode
    private DataLogger Dl;                                  //Datalogger object
    private State state = State.SETRELICARM;                     //Machine State
    private boolean relicSet = false;
    private double currentLauncherPosition = 0;
    private double targetLauncherPosition =0;
    private double launchPosition = 0;
    private double launchPower = .4;
    private double rotateOffset = 0;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    /**
     * Define global variables
     */

    private boolean tel = false;
    private String vuMarkValue = "UNK";
    private double timeout = 0;
    private int count = 1;

    public void runOpMode() {
        /**
         * Setup the init state of the robot.  This configures all the hardware that is defined in
         * the HardwareTestPlatform class.
         */
        robot.init(hardwareMap);

        /**
         *  Create the DataLogger object.
         */
        createDl();

        /**
         * Instantiate the drive class
         */

        DriveMecanum drive = new DriveMecanum(robot, opMode, Dl);

        robot.servoStone.setPosition(.8);                        // move servoStone out of the way
        robot.servoLeft.setPosition(.9);                          // move servoLeft out of the way
        robot.servoRight.setPosition(.185);                         // move servoRight out of the way
        robot.servoRelicGrab.setPosition(0.1);

        robot.motorRelicArm.setPower(.005);
        sleep(75);
        robot.motorRelicArm.setPower(0);

        robot.motorRelicArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRelicArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        /**
         * Start the opMode
         */
        waitForStart();

        robot.servoRelicGrab.setPosition(0.7);

        while (opModeIsActive()) {
            /**
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             * This is the section of code you should change for your robot.
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             */

            switch (state) {
                case SETRELICARM:

                    if(!relicSet) {          // if !relicSet
                        relicSet = true;
                        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                        targetLauncherPosition = 640;

                        telemetry.addData("current Launcher Position = ", currentLauncherPosition);
                        telemetry.addData("target Launcher Position = ", targetLauncherPosition);
                        telemetry.update();

                        sleep(1000);

                        // Lift relic grabber arm up to launching position
                        robot.motorRelicArm.setPower(.2);

                        while (currentLauncherPosition < targetLauncherPosition) {
                            currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                            telemetry.addData("Waiting for  = ", "Relic Arm to Set");
                            telemetry.addData("current Launcher Position = ", currentLauncherPosition);
                            telemetry.addData("target Launcher Position = ", targetLauncherPosition);
                            telemetry.update();

                        }
                        robot.motorRelicArm.setPower(-.05);            // stop the robot relic arm
                        // relic arm should be standing straight up
                        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                        while (currentLauncherPosition > targetLauncherPosition){
                            currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                            telemetry.addData("Waiting for  = ", "Relic Arm to Set");
                            telemetry.addData("current Launcher Position = ", currentLauncherPosition);
                            telemetry.addData("target Launcher Position = ", targetLauncherPosition);
                            telemetry.update();
                        }
                        robot.motorRelicArm.setPower(0);            // stop the robot relic arm
                        // relic arm should be standing straight up

                        launchPosition = 640;
                    }

                    telemetry.addData("current Launcher Position = ", currentLauncherPosition);
                    telemetry.addData("target Launcher Position = ", targetLauncherPosition);
                    telemetry.addData("Launch Position = ", launchPosition);
                    telemetry.addData("Launcher = ", "set!");
                    telemetry.update();

                    state = State.GRIP;
                    break;

                case GRABRELIC:
                    sleep (2000);

                    // Lift relic grabber arm up to launching position
                    robot.motorRelicArm.setPower(launchPower);
                    targetLauncherPosition = 775;
                    currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                    rotateOffset = currentLauncherPosition;

                    while (currentLauncherPosition < targetLauncherPosition){
                        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                        rotateOffset = currentLauncherPosition - rotateOffset;
                        if(rotateOffset >= 10){
                            launchPower = launchPower - 0.002;
                            robot.motorRelicArm.setPower(launchPower);
                        }
                        telemetry.addData("current Launcher Position = ", currentLauncherPosition);
                        telemetry.addData("target Launcher Position = ", targetLauncherPosition);
                        telemetry.addData("Launch Power = ", launchPower);
                        telemetry.addData("Launch Position = ", launchPosition);
                        telemetry.update();
                    }

/*                    robot.motorRelicArm.setPower(.05);

                    currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                    targetLauncherPosition = 800;                   // position where the relic is located

                    while (currentLauncherPosition < targetLauncherPosition){
                        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                        telemetry.addData("current Launcher Position = ", currentLauncherPosition);
                        telemetry.addData("target Launcher Position = ", targetLauncherPosition);
                        telemetry.addData("Launch Position = ", launchPosition);
                        telemetry.addData("Launcher = ", "Grabbing the relic");
                        telemetry.update();
                    }
*/

                    robot.motorRelicArm.setPower(0.004);

                    currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                    targetLauncherPosition = 800;                   // position where the relic is located

                    while (currentLauncherPosition < targetLauncherPosition){
                        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                        telemetry.addData("current Launcher Position = ", currentLauncherPosition);
                        telemetry.addData("target Launcher Position = ", targetLauncherPosition);
                        telemetry.addData("Launch Position = ", launchPosition);
                        telemetry.addData("Launcher = ", "Grabbing the relic");
                        telemetry.update();
                    }

                    robot.motorRelicArm.setPower(-.05);            // slow the robot relic arm down

                    sleep(300);
                    robot.servoRelicGrab.setPosition(0.1);

                    robot.motorRelicArm.setPower(-.1);            // robot relic arm to set position

                    while (currentLauncherPosition > launchPosition){
                        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                        telemetry.addData("current Launcher Position = ", currentLauncherPosition);
                        telemetry.addData("target Launcher Position = ", targetLauncherPosition);
                        telemetry.addData("Launch Position = ", launchPosition);
                        telemetry.addData("Launch = ", "set position");
                        telemetry.update();
                    }

                    robot.motorRelicArm.setPower(0);            // stop the robot relic arm

                    sleep(5000);

                    state = State.HALT;
                    break;

                case GRIP:
                    sleep(3000);
                    robot.servoRelicGrab.setPosition(0.1);
                    sleep(3000);

                    state = State.PLACERELIC;

                    break;

                case PLACERELIC:
                    sleep (2000);
                    sleep (2000);

                    // Lift relic grabber arm up to launching position
                    robot.motorRelicArm.setPower(launchPower);
                    targetLauncherPosition = 775;
                    currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                    rotateOffset = currentLauncherPosition;

                    while (currentLauncherPosition < targetLauncherPosition){
                        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                        rotateOffset = currentLauncherPosition - rotateOffset;
                        if(rotateOffset >= 10){
                            launchPower = launchPower - 0.002;
                            robot.motorRelicArm.setPower(launchPower);
                        }
                        telemetry.addData("current Launcher Position = ", currentLauncherPosition);
                        telemetry.addData("target Launcher Position = ", targetLauncherPosition);
                        telemetry.addData("Launch Power = ", launchPower);
                        telemetry.addData("Launch Position = ", launchPosition);
                        telemetry.update();
                    }

/*                    robot.motorRelicArm.setPower(.05);

                    currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                    targetLauncherPosition = 800;                   // position where the relic is located

                    while (currentLauncherPosition < targetLauncherPosition){
                        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                        telemetry.addData("current Launcher Position = ", currentLauncherPosition);
                        telemetry.addData("target Launcher Position = ", targetLauncherPosition);
                        telemetry.addData("Launch Position = ", launchPosition);
                        telemetry.addData("Launcher = ", "Grabbing the relic");
                        telemetry.update();
                    }
*/

                    robot.motorRelicArm.setPower(0.004);

                    currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                    targetLauncherPosition = 800;                   // position where the relic is located

                    while (currentLauncherPosition < targetLauncherPosition){
                        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                        telemetry.addData("current Launcher Position = ", currentLauncherPosition);
                        telemetry.addData("target Launcher Position = ", targetLauncherPosition);
                        telemetry.addData("Launch Position = ", launchPosition);
                        telemetry.addData("Launcher = ", "Grabbing the relic");
                        telemetry.update();
                    }

                    robot.motorRelicArm.setPower(-.05);            // slow the robot relic arm down

                    sleep(300);
                    robot.servoRelicGrab.setPosition(0.6);

                    sleep(1000);

                    robot.motorRelicArm.setPower(-.05);            // slow the robot relic arm down

                    robot.motorRelicArm.setPower(-.1);            // robot relic arm to set position

                    while (currentLauncherPosition > launchPosition){
                        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                        telemetry.addData("current Launcher Position = ", currentLauncherPosition);
                        telemetry.addData("target Launcher Position = ", targetLauncherPosition);
                        telemetry.addData("Launch Position = ", launchPosition);
                        telemetry.addData("Launch = ", "set position");
                        telemetry.update();
                    }

                    robot.motorRelicArm.setPower(0);            // stop the robot relic arm

                    sleep(5000);

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
        Dl.newLine();
    }

    /**
     * Transmit telemetry.
     */
    private void telemetry() {
        opMode.telemetry.addData("raw ultrasonic", robot.rangeSensor.rawUltrasonic());
        opMode.telemetry.addData("raw optical", robot.rangeSensor.rawOptical());
        opMode.telemetry.addData("cm optical", "%.2f cm", robot.rangeSensor.cmOptical());
        opMode.telemetry.addData("cm", "%.2f cm", robot.rangeSensor.getDistance(DistanceUnit.CM));
        //opMode.telemetry.addData("Right Blue", String.valueOf(robot.colorSensorRight.blue()));
        //opMode.telemetry.addData("Right Red", String.valueOf(robot.colorSensorRight.red()));
        //opMode.telemetry.addData("Left Blue", String.valueOf(robot.colorSensorLeft.blue()));
        //opMode.telemetry.addData("Left Red", String.valueOf(robot.colorSensorLeft.red()));
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
        SETRELICARM, GRABRELIC, PLACERELIC, GRIP, HALT
    }

}
