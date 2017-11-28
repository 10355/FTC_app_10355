/*
    Program:    blueBack
    Team:       10355 - Project Peacock
    Season:     2017-2018 => Relic Recovery
    Autonomous Program - Blue Back Balancing Stone
    Alliance Color: Blue
    Robot Starting Position: Blue balancing stone farthest from the relic mats
    Strategy Description:
        - remove red gem
        - Read encrypto picture
        - Place glyph in the correct column
        - Park in the safe zone position

    Hardware Setup  (NEEDS TO BE UPDATED):
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
        - 1 x Touch sensor - utilized to identify when robot touches wall with the front of the robot
        - 1 x Range Sensor - utilized to position distance from wall during autonomous mode
        - 1 x Motorola Camera - Utilized for decrypting the location of the glyph in autonomous mode

    State Order (NEEDS TO BE UPDATED):
        - ACQUIRECODE                   // Reads image to determine which column to place the glyph
        - REMOVEGEM                     // Determines which gem to remove and knocks it from the mount
        - RETRIEVEGLYPH2                // Goes to the center of the field and retrieves a second
                                        // glyph to place in the glyph box
        - FINDCOLUMN                    // locates the correct position to place the glyph in during
                                        // autonomous mode
        - PLACEGLYPH                    // places the glyphs in the correct column and backs away
                                        // into the safe zone for full points
        - HALT                          // Shutdown sequence for autonomous mode
        - TEST                          // Temporary state for testing specific functions without
                                        // impacting the rest of the program

 */

/*
 * Import the classes we need to have local access to.
 */
package org.firstinspires.ftc.teamcode.Opmodes;

/**
 * Import the classes we need to have local access to.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Autonomous(name = "Auto Back Blue", group = "COMP")

public class blueBack extends LinearOpMode {

    /**
     * Instantiate all objects needed in this class
     */

    private final static HardwareTestPlatform robot = new HardwareTestPlatform();
    private ElapsedTime runtime = new ElapsedTime();        //ElapsedTime
    private LinearOpMode opMode = this;                     //Opmode
    private DataLogger Dl;                                  //Datalogger object
    private String alliance = "blue";                       //Your current alliance
    private State state = State.BALL;                     //Machine State

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
         * Setup Vuforia
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AWLOnnD/////AAAAGUkCQDlQKUEVie7jg6bzwOsIdO360LbYDcYryrOUvM7ISMTrmHv4Z3WRq5IydTQEhQYFOCQhOD6wsaCEHdx3+K/HibQdTtWHzc5xTm//yzcfMcYBwNQsUFGghDV4ccGnbSXHALbYnv63U/n7VeCY91NtLLBe4rB3/U0q22IO6o3Q7Pui+06i3VlTiomIqptoGpbI0kuEwok+6Mq6818ECggYxwpW4UATAy7Rl0eDzp8BzkYEWM8Qe3ykRiEk9D4DBApyx8p3AERmPlQU8rIA/JDAs4tCEJSMNycVw2RKdE1qTrNfVqPe+mYWNOpypVq67odTh7tTHE+BGqdh6znE4NlTia2vr6vmAHjDsQuxn5bm";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        /**
         *  Create the DataLogger object.
         */
        createDl();

        /**
         * Calibrate the MR Gyro
         */
        robot.sensorGyro.calibrate();
        while (robot.sensorGyro.isCalibrating()) {
            telemetry.addData("Waiting on Gyro Calibration", "");
            telemetry.update();
        }

        /**
         * Instantiate the drive class
         */

        DriveMecanum drive = new DriveMecanum(robot, opMode, Dl);

        /**
         * Deploy the color sensor
         */

        robot.servoLeft.setPosition(1);
        robot.servoRight.setPosition(0);

        sleep(1000);

        telemetry.addData("Color Sensor Red = ", robot.colorSensorRight.red());
        telemetry.addData("Color Sensor Blue = ", robot.colorSensorRight.blue());
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        /**
         * Start the opMode
         */
        waitForStart();

        relicTrackables.activate();     //Start Relic Tracking

        while (opModeIsActive()) {
            /**
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             * This is the section of code you should change for your robot.
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             */

            switch (state) {
                case TEST:
                    drive.translateRange(.5, 0, 35);
                    state = State.HALT;
                    break;
                case BALL:
                    telemetry.addData("VUMARK", String.valueOf(vuMarkValue));
                    telemetry.update();

                    robot.servoRight.setPosition(.5);
                    telemetry.addData("VUMARK", String.valueOf(vuMarkValue));
                    telemetry.addData("SERVO position", robot.servoLeft.getPosition());
                    telemetry.update();
                    sleep(1000);
                    if (robot.colorSensorLeft.blue() > robot.colorSensorLeft.red()) {  //Blue is back
                        drive.translateTime(.8, .2, 180);
                    }
                    else {
                        drive.translateTime(.8, .2, 0);
                    }

                    robot.servoRight.setPosition(0);

                    state = State.CHECK_VU;             //temporary code to bypass vuforia
                    break;

                case CHECK_VU:
                    telemetry.addData("VUMARK", String.valueOf(vuMarkValue));
                    telemetry.update();

                    if (vuMarkValue == "LEFT") {
                        state = State.LEFT;
                    }
                    if (vuMarkValue == "CENTER") {
                        state = State.CENTER;
                    }
                    if (vuMarkValue == "RIGHT") {
                        state = State.RIGHT;
                    }
                    break;

                case LEFT:
                    telemetry.addData("VUMARK", String.valueOf(vuMarkValue));
                    telemetry.update();

                    drive.translateRange(.2, 180, 18);
                    telemetry.addData("Range", String.valueOf(robot.rangeSensor.cmUltrasonic()));
                    telemetry.addData("Action = ", "Drive forward");
                    telemetry.update();
                    sleep(1000);

                    drive.translateRange(.2, 270, 15);
                    telemetry.addData("Range", String.valueOf(robot.rangeSensor.cmUltrasonic()));
                    telemetry.addData("Action = ", "strafe right #1");
                    telemetry.update();
                    sleep(1000);

                    drive.translateTime(1.5, .2, 270);
                    telemetry.addData("Action = ", "strafe right #2");
                    telemetry.update();
                    sleep(1000);

                    robot.servoBlockExit.setPosition(.5);
                    telemetry.addData("Action = ", "Kick block");

                    drive.translateTime(.75, .2, 0);
                    telemetry.addData("Action = ", "drive backward & Halt");

                    sleep(500);
                    drive.translateTime(1, .2, 180);
                    telemetry.addData("Action = ", "drive backward & Halt");

                    drive.translateTime(.5, .2, 0);
                    telemetry.addData("Action = ", "drive backward & Halt");
                    telemetry.update();

                    state = State.HALT;

                    break;

                case CENTER:
                    telemetry.addData("VUMARK", String.valueOf(vuMarkValue));
                    telemetry.update();

                    drive.translateRange(.2, 180, 18);
                    telemetry.addData("Action = ", "Drive forward");
                    telemetry.addData("Range", String.valueOf(robot.rangeSensor.cmUltrasonic()));
                    telemetry.update();
                    sleep(1000);

                    drive.translateRange(.2, 270, 15);
                    telemetry.addData("Action = ", "strafe right #1");
                    telemetry.addData("Range", String.valueOf(robot.rangeSensor.cmUltrasonic()));
                    telemetry.update();
                    sleep(1000);

                    drive.translateTime(2   , .2, 270);
                    telemetry.addData("Action = ", "strafe right for 2 seconds");
                    telemetry.update();
                    sleep(1000);

                    drive.translateRange(.2, 180, 18);
                    telemetry.addData("Action = ", "Drive forward to make sure we haven't drifted too far back");
                    telemetry.addData("Range", String.valueOf(robot.rangeSensor.cmUltrasonic()));
                    telemetry.update();
                    sleep(1000);

                    drive.translateRange(.2, 270, 13);
                    telemetry.addData("Action = ", "strafe right to find the next fin");
                    telemetry.addData("Range", String.valueOf(robot.rangeSensor.cmUltrasonic()));
                    telemetry.update();
                    sleep(1000);

                    drive.translateTime(1.5, .2, 270);
                    telemetry.addData("Action = ", "strafe right to position block");
                    telemetry.update();
                    sleep(1000);

//                    drive.translateRange(.2, 180, 10);
//                    telemetry.addData("Action = ", "Drive forward");
//                    telemetry.update();

                    robot.servoBlockExit.setPosition(.5);
//                    sleep(200);
//                    robot.servoBlockExit.setPosition(1);
                    telemetry.addData("Action = ", "Kick block");

                    drive.translateTime(1, .2, 0);
                    telemetry.addData("Action = ", "drive backward & Halt");

                    sleep(500);
                    drive.translateTime(1, .2, 180);
                    telemetry.addData("Action = ", "drive backward & Halt");

                    drive.translateTime(.5, .2, 0);
                    telemetry.addData("Action = ", "drive backward & Halt");
                    telemetry.update();

                    state = State.HALT;

                    break;

                case RIGHT:
                    telemetry.addData("VUMARK", String.valueOf(vuMarkValue));
                    telemetry.update();

                    drive.translateRange(.2, 180, 18);
                    telemetry.addData("Range", String.valueOf(robot.rangeSensor.cmUltrasonic()));
                    telemetry.addData("Action = ", "Drive forward");
                    telemetry.update();
                    sleep(1000);

                    drive.translateRange(.2, 270, 15);
                    telemetry.addData("Range", String.valueOf(robot.rangeSensor.cmUltrasonic()));
                    telemetry.addData("Action = ", "strafe right #1");
                    telemetry.update();
                    sleep(1000);

                    drive.translateTime(5, .2, 270);
                    telemetry.addData("Action = ", "strafe right #2");
                    telemetry.update();
                    sleep(1000);

                    drive.translateRange(.2, 180, 18);
                    telemetry.addData("Range", String.valueOf(robot.rangeSensor.cmUltrasonic()));
                    telemetry.addData("Action = ", "Drive forward");
                    telemetry.update();
                    sleep(1000);

                    drive.translateRange(.2, 270, 15);
                    telemetry.addData("Range", String.valueOf(robot.rangeSensor.cmUltrasonic()));
                    telemetry.addData("Action = ", "strafe right #1");
                    telemetry.update();
                    sleep(1000);

                    drive.translateTime(1.5, .2, 270);
                    telemetry.addData("Action = ", "strafe right #2");
                    telemetry.update();
                    sleep(1000);

                    robot.servoBlockExit.setPosition(.5);
                    telemetry.addData("Action = ", "Kick block");

                    drive.translateTime(.75, .2, 0);
                    telemetry.addData("Action = ", "drive backward & Halt");

                    sleep(500);
                    drive.translateTime(1, .2, 180);
                    telemetry.addData("Action = ", "drive backward & Halt");

                    drive.translateTime(.5, .2, 0);
                    telemetry.addData("Action = ", "drive backward & Halt");
                    telemetry.update();

                    state = State.HALT;

                    break;

                case DRIVE:
                    robot.motorLift.setPower(1);
                    sleep(500);
                    robot.motorLift.setPower(0);

                    robot.servoLeft.setPosition(1);
                    telemetry.addData("SERVO position", robot.servoLeft.getPosition());
                    telemetry.update();
                    sleep(2000);
                    if (robot.colorSensorLeft.blue() > robot.colorSensorLeft.red()) {  //Blue is back
                        drive.translateTime(.4, .5, 180);
                    }
                    else {
                        drive.translateTime(.4, .5, 0);
                    }

                    robot.servoLeft.setPosition(0);

                    drive.translateRange(.2, 180, 35);

                    drive.translateRange(.2, 270, 30);

                    drive.translateRange(.5, 180, 13);

                    drive.translateTime(.5, .5, 0);

                    state = State.HALT;

                    break;

                case VUMark:
                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

                    if (count == 1) {  //Only do this the first time
                        timeout = 10;
                        timeout = opMode.getRuntime() + timeout;
                        count = count + 1;
                    }

                    /**
                     * Look for the VuMark for the specified timeout and return the vuMarkValue
                     * variable for use later in the program.
                     */
                    if (vuMark == RelicRecoveryVuMark.UNKNOWN  &&
                            opMode.getRuntime() < timeout) {

                        vuMarkValue = String.valueOf(vuMark);  //Assign the vuMarkValue variable

                        telemetry.addData("timeout", String.valueOf(timeout));
                        telemetry.addData("now", String.valueOf(opMode.getRuntime()));
                        telemetry.addData("vuMarkValue ", vuMarkValue);

                        /**
                         * See if any of the instances of {@link relicTemplate} are currently visible.
                         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
                         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
                         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
                         */

                        telemetry.addData("VuMark", "%s visible", vuMark);

                        telemetry.update();
                    } else {
                        vuMarkValue = String.valueOf(vuMark);
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        telemetry.addData("vuMarkValue ", vuMarkValue);
                        telemetry.update();
//                        sleep(1000);
                        state = State.BALL;  //The vuMark was found so move on to the next state
                    }

                    break;

                case DISPLAY:
                    telemetry.addData("vuMarkValue", String.valueOf(vuMarkValue));
                    telemetry.update();
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
        Dl.addField(String.valueOf(alliance));
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
        HALT, DRIVE, TAIL, COLOR_SENSOR, RANGE, VUMark, DISPLAY, LEFT, BALL, CHECK_VU, CENTER,
        RIGHT, TEST
    }

}
