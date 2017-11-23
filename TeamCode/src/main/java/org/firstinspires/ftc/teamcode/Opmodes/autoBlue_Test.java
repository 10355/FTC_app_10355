package org.firstinspires.ftc.teamcode.Opmodes;

/*
    Team:       10355 - Project Peacock
    Season:     2017-2018 => Relic Recovery
    Autonomous Program - Red Strategy #1
    Alliance Color: Red
    Robot Starting Position: Red balancing stone closest to relic mats
    Strategy Description:
        - remove blue gem
        - Read encrypto picture
        - Place glyph in the correct column
        - Park in the center position

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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;
import org.firstinspires.ftc.teamcode.Libs.VuforiaLib;

/**
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "Auto Blue 1 ", group = "COMP")

public class autoBlue_Test extends LinearOpMode {

    /**
     * Instantiate all objects needed in this class
     */

    private final static HardwareTestPlatform robot = new HardwareTestPlatform();
    double radians = 0;
    private VuforiaLib myVuforia = new VuforiaLib();
    private ElapsedTime runtime = new ElapsedTime();
    /**
     * Define global variables
     */
    private double mm = 500;            //Distance for translateDistance method
    private double power = .6;          //Motor power for all methods
    private double distance = 0;        //Distance in cm that the robot will travel for translateDistance() function
    private double heading = 90;        //Heading for all methods
    private double y = -200;            //Vuforia y stop coordinate
    private double x = -200;            //Vuforia x stop coordinate
    private double changeSpeed = 0;     //Rotation speed for motor speed calculations
    private double initZ;               //The initial reading from integrated Z
    private double currentZint;         //Current integrated Z value
    private double zCorrection;         //Course correction in degrees
    private double timeOut = 5;         //Timeout in seconds for translateTime method
    private double timeOutTime;         //Calculated time to stop
    private String procedure;           //Name of the method that is running
    private double ods = 0;             //Value returned from the Optical Distance Sensor
    private double colorRightRed = 0;   //Value from color sensor
    private double colorRightBlue = 0;  //Value from color sensor
    private double colorLeftRed = 0;    //Value from color sensor
    private double colorLeftBlue = 0;   //Value from color sensor
    private double LF, RF, LR, RR;      //Motor power setting
    private double myCurrentMotorPosition;  //Current encoder position
    private double myTargetPosition;        //Target encoder position
    private DataLogger Dl;                          //Datalogger object
    private double motorCorrectCoefficient = .05;    //Amount to divide the zInt drift by
    private String alliance = "red";                //Your current alliance
    private String color = "";
    private autoBlue_Test.State state = autoBlue_Test.State.ACQUIRECODE;    //Machine State


    public static final String TAG = "Vuforia VuMark Sample";
    public static final double COUNTS_PER_MM = 3.509;

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    public void runOpMode() {

                /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AWLOnnD/////AAAAGUkCQDlQKUEVie7jg6bzwOsIdO360LbYDcYryrOUvM7ISMTrmHv4Z3WRq5IydTQEhQYFOCQhOD6wsaCEHdx3+K/HibQdTtWHzc5xTm//yzcfMcYBwNQsUFGghDV4ccGnbSXHALbYnv63U/n7VeCY91NtLLBe4rB3/U0q22IO6o3Q7Pui+06i3VlTiomIqptoGpbI0kuEwok+6Mq6818ECggYxwpW4UATAy7Rl0eDzp8BzkYEWM8Qe3ykRiEk9D4DBApyx8p3AERmPlQU8rIA/JDAs4tCEJSMNycVw2RKdE1qTrNfVqPe+mYWNOpypVq67odTh7tTHE+BGqdh6znE4NlTia2vr6vmAHjDsQuxn5bm";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
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

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        /**
         * Setup the init state of the robot.  This configures all the hardware that is defined in
         * the HardwareTestPlatform class.
         */
        robot.init(hardwareMap);

        /**
         * Set the initial servo positions
         */
        robot.servoLeft.setPosition(0);
        robot.servoRight.setPosition(0);

        /**
         *  Create the DataLogger object.
         */
        createDl();

        /**
         * Calibrate the gyro
         */
        robot.sensorGyro.calibrate();
        while (robot.sensorGyro.isCalibrating()) {
            telemetry.addData("Waiting on Gyro Calibration", "");
            telemetry.update();
        }

        telemetry.addData(">", "Ready");
        telemetry.update();

        /**
         * Start the opMode
         */
        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {
            /**
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             * This is the section of code you should change for your robot.
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             */
            // FOO
            switch (state) {
                case ACQUIRECODE:

                    /**
                     * See if any of the instances of {@link relicTemplate} are currently visible.
                     * {@link RelicRecoveryVuMark} is an enum which can have the following values:
                     * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
                     * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
                     */
                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                        telemetry.addData("VuMark", "%s visible", vuMark);

                        // set the next state to knock the blue gem off
                        state = State.REMOVEGEM;
                        //Exit the OpMode
                    }
                    else {
                        telemetry.addData("VuMark", "not visible");
                    }

                    telemetry.update();

                    break;

                case REMOVEGEM:
                    robot.servoLeft.setPosition(.45);        // Place the left arm between the gems
                    getColor();

                    if (color  == "blue"){
                        heading = 180;              // Direction to move
                        power = .5;                  // apply full power
                        timeOut = .2;               // backup for 1/2 second
                        translateTime();            // apply parameters

                        robot.servoLeft.setPosition(0);  // Raise arm

                        heading = 0;              // Direction to move
                        power = 1;                  // apply full power
                        timeOut = .75;               // backup for 1/2 second
                        translateTime();            // apply parameters

                        telemetry.addData("Detected ", "RED");

                    } else if (color == "red"){
                        heading = 0;              // Direction to move
                        power = .5;                  // apply full power
                        timeOut = .25;               // backup for 1/2 second
                        translateTime();            // apply parameters
                        telemetry.addData("Detected ", "BLUE");

                        robot.servoLeft.setPosition(0);  // Raise arm

                        heading = 0;              // Direction to move
                        power = .5;                  // apply full power
                        timeOut = .25;               // backup for 1/2 second
                        translateTime();            // apply parameters
                    }


                    telemetry.update();

//                    sleep(25000);                // Pause for 1 second once the white line is located to allow
                    // for the Vuforia to acquire positioning data

                    state = State.RETRIEVEGLYPH2;         // Change to the next state
                    break;

                case TEST:

                    distance = 100;             // distance in cm for how far forward to go.
                    power = .5;
                    translateDistance();

                    state = State.HALT;         // Change to the next state

                    break;

                case RETRIEVEGLYPH2:

                    // drive off of the balancing stone
                    heading = 0;                // Set heading in forward direction
                    power = 1;                  // apply full power
                    timeOut = .5;               // drive for 1/2 second
                    translateTime();            // apply parameters

                    //rotate the robot 90 degrees to align with glyph and glyph box
                    heading = 180;                // set heading 90 degrees from current position => may need to adjust based on gyro tolerance
                    power = .5;                  // apply partial power to help control turn
                    pivot ();                    // align the robot to the glyphs/crypto box

                    // set the arm to pick up a second glyph
                    setArm();

                    // retrieve the glyph and place in glyph loader
                    loadGlyph();

                    //
                    state = State.HALT;     // put both glyphs in the correct column
                    break;

                case FINDCOLUMN:
                    // Position the robot in starting position (next to the balancing stone
                    heading = 180;                // Direction to move
                    power = 1;                  // apply full power
                    timeOut = 2;               // backup for 1/2 second
                    translateTime();            // apply parameters

                    heading = 270;                // Direction to move
                    power = 1;                  // apply full power
                    timeOut = .25;               // backup for 1/2 second
                    translateTime();            // apply parameters


                    // drive towards the cryptobox using the distance sensor
                    heading = 180;                // Direction to move
                    power = .5;                  // apply full power
                    translateSensorDistance();   // apply parameters

                    break;

                case PLACEGLYPH:              // State to place the glyph in the cryptobox


                    heading = 0;                // set heading towards glyphs
                    power = 1;                  // apply full power
                    timeOut = 2;                // drive for 2 seconds
                    translateSensorDistance(); // apply parameters

                    break;

                case HALT:
                    motorsHalt();               //Stop the motors

                    //Stop the DataLogger
                    dlStop();

                    //Exit the OpMode
                    requestOpModeStop();
                    break;
            }
            /**
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             * Don't change anything past this point.  Bad things could happen.
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             */
        }
    }


    private void getColor() {
        color = "unk";
        double red;
        double blue;

        sleep(100);

        while (color.equals("unk")) {
            red = robot.colorSensorLeft.red();
            blue = robot.colorSensorLeft.blue();

            if (red >= 2) {
                color = "red";
            }
            if (blue >= 2) {
                color = "blue";
            }

            telemetry.addData("Red Value = ", red);
            telemetry.addData("Blue Value = ", blue);
            telemetry.addData("Color Value = ", color);
            telemetry.update();

            idle();
        }
    }

    /**
     * Translate on a heading for a defined period of time.
     */

/**
    private void translateTime() {
        procedure = "translateTime";
        initZ = robot.mrGyro.getIntegratedZValue();
        getSensorData();

        radians = heading * (Math.PI / 180);
        timeOutTime = runtime.time() + timeOut;

        while (opModeIsActive() && runtime.time() < timeOutTime) {
            LF = calcLF();
            RF = calcRF();
            LR = calcLR();
            RR = calcRR();

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                zCorrection = Math.abs(initZ - currentZint);

                courseCorrect();
            } else {
                zCorrection = 0;
            }

            setPower();

            myCurrentMotorPosition = robot.motorLF.getCurrentPosition();

            getSensorData();

            telemetry();
            logData();

            idle();
        }
        motorsHalt();
    }

**/
    private void translateTime() {
        procedure = "translateTime";
        getSensorData();

        telemetry.addData("Retrieved Sensor Data = ", "True");
        telemetry.update();

        sleep(1000);

        radians = heading * (Math.PI / 180);
        timeOutTime = runtime.time() + timeOut;

        LF = calcLF();
        RF = calcRF();
        LR = calcLR();
        RR = calcRR();

        setPower();

        telemetry.addData("Set Power = ", "True");
        telemetry.update();

        sleep(1000);

        while (opModeIsActive() && runtime.time() < timeOutTime) {

//            telemetry();
            logData();

            idle();
        }
        motorsHalt();
    }

    /**
     * Pivot the robot to a new heading. 0 is straight ahead, 1 to 179 is to the left -1 to -179 is
     * to the right.
     */
    /**
    private void pivot() {
        procedure = "Pivot";
        initZ = robot.mrGyro.getIntegratedZValue();

        telemetry.addData("Gyro Value = ", initZ);
        telemetry.update();

        sleep(20000);

        if (heading > 0) {
            while (currentZint < heading && opModeIsActive()) {
                LR = -power;
                LF = -power;
                RR = power;
                RF = power;

                setPower();

                getSensorData();

                logData();
                idle();
            }

            if (heading < 0 && opModeIsActive()) {
                while (currentZint < heading) {
                    LR = power;
                    LF = power;
                    RR = -power;
                    RF = -power;

                    setPower();

                    getSensorData();

                    logData();
                    idle();
                }
            }
        }
        motorsHalt();
    }
**/

    /**
     * Pivot the robot to a new heading. 0 is straight ahead, 1 to 179 is to the left -1 to -179 is
     * to the right.
     */
    private void pivot() {
        procedure = "Pivot";
        initZ = robot.mrGyro.getIntegratedZValue();

        telemetry.addData("Gyro Value = ", initZ);
        telemetry.update();

        sleep(5000);

        LR = -power;
        LF = -power;
        RR = power;
        RF = power;

        setPower();

        while (currentZint < heading && opModeIsActive()) {

            getSensorData();

            telemetry.addData("currentZint Value = ", currentZint);
            telemetry.update();

            logData();
            idle();
        }
        motorsHalt();
        sleep(5000);
    }

    /**
     * Translate on a heading the distance specified in cm.  Will convert to mm within the function.
     */
    private void translateDistance() {
        procedure = "translateDistance";
        initZ = robot.mrGyro.getIntegratedZValue();
        myCurrentMotorPosition = robot.motorLF.getCurrentPosition();
        private double mmDistance = distance * 10;

        //  Need to determine COUNTS_PER_MM configuration
        myTargetPosition = myCurrentMotorPosition + (mmDistance * COUNTS_PER_MM );

        getSensorData();

        radians = heading * (Math.PI / 180);

        while (opModeIsActive() && myTargetPosition > myCurrentMotorPosition) {
            LF = calcLF();
            RF = calcRF();
            LR = calcLR();
            RR = calcRR();

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                zCorrection = Math.abs(initZ - currentZint);

                courseCorrect();
            } else {
                zCorrection = 0;
            }

            setPower();

            myCurrentMotorPosition = robot.motorLF.getCurrentPosition();

            getSensorData();

            telemetry();
            logData();

            idle();
        }
        motorsHalt();               // shut down all of the motors before exiting function
    }

    /**
     * Translate on a heading for a defined period of time.
     */
    private void translateSensorDistance() {
        procedure = "translateDistance";
        initZ = robot.mrGyro.getIntegratedZValue();
        getSensorData();

        radians = heading * (Math.PI / 180);
        timeOutTime = runtime.time() + timeOut;

        while (opModeIsActive() && runtime.time() < timeOutTime) {
            LF = calcLF();
            RF = calcRF();
            LR = calcLR();
            RR = calcRR();

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                zCorrection = Math.abs(initZ - currentZint);

                courseCorrect();
            } else {
                zCorrection = 0;
            }

            setPower();

            myCurrentMotorPosition = robot.motorLF.getCurrentPosition();

            getSensorData();

            telemetry();
            logData();

            idle();
        }
        motorsHalt();
    }

    /**
     * Calculate the wheel speeds.
     *
     * @return wheel speed
     */
    private double calcLF() {
        LF = power * Math.sin(radians + (Math.PI / 4)) + changeSpeed;

        if (LF > 1 || LF < -1) {
            LF = 0;
        }
        return LF;
    }

    private double calcRF() {
        RF = power * Math.cos(radians + (Math.PI / 4)) - changeSpeed;

        if (RF > 1 || RF < -1) {
            RF = 0;
        }
        return RF;
    }

    private double calcLR() {
        LR = power * Math.cos(radians + (Math.PI / 4)) + changeSpeed;

        if (LR > 1 || LR < -1) {
            LR = 0;
        }
        return LR;
    }

    private double calcRR() {
        RR = power * Math.sin(radians + (Math.PI / 4)) - changeSpeed;

        if (RR > 1 || RR < -1) {
            RR = 0;
        }
        return RR;
    }

    private void getSensorData() {
        colorRightRed = robot.colorSensorRight.red();
        colorRightBlue = robot.colorSensorRight.blue();
        colorLeftRed = robot.colorSensorLeft.red();
        colorLeftBlue = robot.colorSensorLeft.blue();
//        ods = robot.ods.getLightDetected();
        currentZint = robot.mrGyro.getIntegratedZValue();
    }

    private void createDl() {

        Dl = new DataLogger("AutoMecanumSimpleTest" + runtime.time());
        Dl.addField("runTime");
        Dl.addField("Alliance");
        Dl.addField("State");
        Dl.addField("Procedure");
        Dl.addField("courseCorrect");
        Dl.addField("heading");
        Dl.addField("X");
        Dl.addField("Y");
        Dl.addField("initZ");
        Dl.addField("currentZ");
        Dl.addField("zCorrection");
        Dl.addField("touchSensor");
        Dl.addField("ODS");
        Dl.addField("colorRightRed");
        Dl.addField("colorRightBlue");
        Dl.addField("colorLeftRed");
        Dl.addField("colorLeftBlue");
        Dl.addField("LFTargetPos");
        Dl.addField("LFMotorPos");
        Dl.addField("LF");
        Dl.addField("RF");
        Dl.addField("LR");
        Dl.addField("RR");
        Dl.newLine();
    }

    private void logData() {

        Dl.addField(String.valueOf(runtime.time()));
        Dl.addField(String.valueOf(alliance));
        Dl.addField(String.valueOf(state));
        Dl.addField(String.valueOf(procedure));
//        Dl.addField(String.valueOf(courseCorrect));
        Dl.addField(String.valueOf(heading));
        Dl.addField(String.valueOf(x));
        Dl.addField(String.valueOf(y));
        Dl.addField(String.valueOf(initZ));
        Dl.addField(String.valueOf(currentZint));
        Dl.addField(String.valueOf(zCorrection));
        Dl.addField(String.valueOf(robot.touchSensor.getValue()));
        Dl.addField(String.valueOf(ods));
        Dl.addField(String.valueOf(colorRightRed));
        Dl.addField(String.valueOf(colorRightBlue));
        Dl.addField(String.valueOf(colorLeftRed));
        Dl.addField(String.valueOf(colorLeftBlue));
        Dl.addField(String.valueOf(myTargetPosition));
        Dl.addField(String.valueOf(robot.motorLF.getCurrentPosition()));
        Dl.addField(String.valueOf(LF));
        Dl.addField(String.valueOf(RF));
        Dl.addField(String.valueOf(LR));
        Dl.addField(String.valueOf(RR));
        Dl.newLine();
    }

    /**
     * Stop the DataLogger
     */
    private void dlStop() {
        Dl.closeDataLogger();

    }

    /**
     * Transmit telemetry.
     */
    private void telemetry() {

        telemetry.addData("Alliance", String.valueOf(alliance));
        telemetry.addData("State", String.valueOf(state));
        telemetry.addData("Procedure", String.valueOf(procedure));
        telemetry.addData("Heading", String.valueOf(heading));
        telemetry.addData("Target X", String.valueOf(x));
        telemetry.addData("Target Y", String.valueOf(y));
        telemetry.addData("Current Z Int", String.valueOf(currentZint));
        telemetry.addData("Z Correction", String.valueOf(zCorrection));
        telemetry.addData("touchSensor", String.valueOf(robot.touchSensor.getValue()));
        telemetry.addData("ODS", String.valueOf(ods));
        telemetry.addData("Color Right Red", String.valueOf(colorRightRed));
        telemetry.addData("Color Right Blue", String.valueOf(colorRightBlue));
        telemetry.addData("Color Left Red", String.valueOf(colorLeftRed));
        telemetry.addData("Color Left Blue", String.valueOf(colorLeftBlue));
        telemetry.addData("Target Encoder Position", String.valueOf(myTargetPosition));
        telemetry.addData("Current Encoder Position", String.valueOf(robot.motorLF.getCurrentPosition()));
        telemetry.addData("LF", String.valueOf(LF));
        telemetry.addData("RF", String.valueOf(RF));
        telemetry.addData("LR", String.valueOf(LR));
        telemetry.addData("RR", String.valueOf(RR));
        telemetry.update();
    }

    /**
     * Set the arm position to pick up a second glyph
     */
    private void setArm() {

        // place the lift motor at 90 degree angle to floor
        robot.motorLift.setTargetPosition(90);

        robot.servoLiftLeft.setPosition(.6);
        robot.servoLiftRight.setPosition(.6);

    }

    private void loadGlyph() {

        // place the lift motor at 90 degree angle to floor
        robot.servoLiftLeft.setPosition(.45);
        robot.servoLiftRight.setPosition(.45);

        robot.motorLift.setTargetPosition(110);

        robot.servoLiftLeft.setPosition(.6);
        robot.servoLiftRight.setPosition(.6);
    }

    /**
     * Change power to the motors to correct for heading drift as indicated by the gyro.
     */
    private void courseCorrect() {
        if (currentZint > initZ) {  //Robot has drifted left
            LF = LF + motorCorrectCoefficient;
            LR = LR + motorCorrectCoefficient;
            RF = RF - motorCorrectCoefficient;
            RR = RR - motorCorrectCoefficient;
        }

        if (currentZint < initZ) {  //Robot has drifted right
            LF = LF - motorCorrectCoefficient;
            LR = LR - motorCorrectCoefficient;
            RF = RF + motorCorrectCoefficient;
            RR = RR + motorCorrectCoefficient;
        }
    }

    /**
     * Set the power level of the motors.
     */
    private void setPower() {
        robot.motorLF.setPower(LF);
        robot.motorRF.setPower(RF);
        robot.motorLR.setPower(LR);
        robot.motorRR.setPower(RR);
    }

    /**
     * Set the power level of the motors.
     */
    private void motorsHalt() {
        robot.motorLF.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    /**
     * Enumerate the States of the machine.
     */
    enum State {
        ACQUIRECODE, REMOVEGEM, RETRIEVEGLYPH2, FINDCOLUMN, PLACEGLYPH, HALT, TEST
    }

}
