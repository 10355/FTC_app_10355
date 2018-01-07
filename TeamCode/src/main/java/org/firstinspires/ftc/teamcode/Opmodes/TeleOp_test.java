/*
    Program:    TeleOp_Test.java
    Opmode Name: TeleOp_Test
    Team:       10355 - Project Peacock
    Season:     2017-2018 => Relic Recovery
    Autonomous Program - Teleop Mode
    Alliance Color: N/A
    Robot Starting Position: N/A
    Strategy Description:
        - Place Glyphs
        - Retrieve and place relic
        - Park on balancing stene

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

 */

package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 * <p>
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Teleop_Test", group = "COMP")

public class TeleOp_test extends LinearOpMode {
    /**
     * Instantiate all objects needed in this class
     */
    private final static HardwareTestPlatform robot = new HardwareTestPlatform();
    double currentLauncherPosition = 0;
    double currentGlyphArmPosition = 0;
    double rangeDistance=0;
    double targetLauncherPosition = 0;
    double launchPosition = 0;
    double currentSlidePosition = 0;
    double relicRightPosition = 1;
    double triggerPower = 0;
    boolean relicSet = false;              // flag to identify if the relic arm has been deployed
    boolean relicDeploy = false;             // flag to identify if the relic arm is deployed
    boolean relicCaptured = false;          // flag to identify if the relic is captured

    public static final double RELICSETPOSITION = 640;
    public static final double MINARMPOSITION = -520;
    public static final double TARGETARMPOSITION = -500;
    public static final double MAXARMPOSITION = -10;

    @Override
    public void runOpMode() {

        begin();

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.servoStone.setPosition(.8);                        // move servoStone out of the way
        robot.servoLeft.setPosition(.9);                          // move servoLeft out of the way
        robot.servoRight.setPosition(.1);                         // move servoRight out of the way
        robot.servoRelicGrab.setPosition(0.1);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Waiting for Teleop to start.", "");    //
        telemetry.update();
        waitForStart();

        robot.servoRelicGrab.setPosition(0.5);

        telemetry.addData("Teleop Test Active", "");    //
        telemetry.update();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /**
             *  Gamepad1 Controls
             *      -   Drive & Strafe          =    Gamepad1.Left_Stick
             *      -   Drive & rotate          =    Gamepad1.Right_Stick
             *      -   Autopark                =    Gamepad1.x
             *      -   Speed Forward           =    Gamepad1.left_bumper
             *      -   Speed Backward          =    Gamepad1.right_bumper
             */

            double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y *-1);
            double robotAngle = Math.atan2((gamepad1.left_stick_y*-1), (gamepad1.left_stick_x )) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            telemetry.addData("Speed Value = ", speed);
            telemetry.addData("Robot Angle", robotAngle);
            telemetry.addData("rightX", rightX);

            final double vlf = speed * Math.cos(robotAngle) + rightX;
            final double vrf = speed * Math.sin(robotAngle) - rightX;
            final double vlr = (speed * Math.sin(robotAngle) + rightX)*.9;
            final double vrr = (speed * Math.cos(robotAngle) - rightX)*.9;

            robot.motorLF.setPower(vlf);
            robot.motorRF.setPower(vrf);
            robot.motorLR.setPower(vlr);
            robot.motorRR.setPower(vrr);

            /**
             * Park the robot on the balancing stone
             */
            if(gamepad1.x == true) {
                robot.motorLF.setPower(.75);
                robot.motorRF.setPower(.75);
                robot.motorLR.setPower(.75);
                robot.motorRR.setPower(.75);

                sleep(1100);

                motorsHalt();
            }  // if gamepad1.x

            if (gamepad1.left_bumper){      // if gamepad1.left_bumper
                rangeDistance = robot.rangeSensor.cmUltrasonic();
                while (rangeDistance > 40){         // while rangeDistance
                    // lower arm into the correct position

                    // raise arm into position to place glyphs in loader
                    currentGlyphArmPosition = robot.motorLift.getCurrentPosition();
                    if (currentGlyphArmPosition > MAXARMPOSITION){
                        robot.motorLift.setPower(.2);
                    }else if (currentGlyphArmPosition > MINARMPOSITION){
                        robot.motorLift.setPower(0);
                    }

                    // drive fulll speed towards glyphs
                    robot.motorLF.setPower(-1);
                    robot.motorLR.setPower(-1);
                    robot.motorRF.setPower(-1);
                    robot.motorRR.setPower(-1);
                    rangeDistance = robot.rangeSensor.cmUltrasonic();
                }               // while rangeDistance
                motorsHalt();
            }           // if gamepad1.left_bumper

            if (gamepad1.right_bumper) {     // if gamepad1.right_bumper

                rangeDistance = robot.rangeSensor.cmUltrasonic();
                while (rangeDistance < 100) {       // while rangeDistance

                    // lower arm into the correct position

                    currentGlyphArmPosition = robot.motorLift.getCurrentPosition();
                    if (currentGlyphArmPosition < MINARMPOSITION){
                        robot.motorLift.setPower(.1);
                    }else if (currentGlyphArmPosition > TARGETARMPOSITION){
                        robot.motorLift.setPower(-.2);
                        robot.servoLiftLeft.setPosition (0.25);
                        robot.servoLiftRight.setPosition(0.75);
                    } else {
                        robot.motorLift.setPower(0);
                    }

                    // drive fulll speed towards glyphs
                    robot.motorLF.setPower(1);
                    robot.motorLR.setPower(1);
                    robot.motorRF.setPower(1);
                    robot.motorRR.setPower(1);
                    rangeDistance = robot.rangeSensor.cmUltrasonic();
                }                   // while rangeDistance

                motorsHalt();
            }               // if gamepad1.right_bumper
            /**
             *  Gamepad2 Controls
             *      -   Block ejector   =   gamepad2.b
             *      -   Tail controls   =   gamepad2.right_trigger
             *      -   Tail controls   =   gamepad2.left_trigger
             *      -   Linear Slide Up =   gamepad2.y
             *      -   Linear Slide Dn =   gamepad2.a
             *      -   Glyph arm control   =   gamepad2.right_stick
             */

            if (gamepad1.b || gamepad2.b) {           // if gamepad1.x
                robot.servoBlockExit.setPosition(.5);
                sleep(400);
            } else{
                robot.servoBlockExit.setPosition(.95);
            }           // if gamepad1.x

            if ((gamepad2.right_bumper) && (relicRightPosition < 1)) {
                relicRightPosition = 1;
                robot.servoLiftRight.setPosition(relicRightPosition);
                robot.servoLiftLeft.setPosition(0);
                sleep(200);
            }  else if ((gamepad2.right_bumper) && (relicRightPosition > .8)) {
                relicRightPosition = 0.75;
                robot.servoLiftRight.setPosition(relicRightPosition);
                robot.servoLiftLeft.setPosition(.25);
                sleep(200);
            }  // gamepad1.left_bumper

            if (gamepad2.a) {
                // lower Glyph control arm
                // need to get it out of the way so that
                currentGlyphArmPosition = robot.motorLift.getCurrentPosition();
                while (currentGlyphArmPosition > (TARGETARMPOSITION/2)) {
                    currentGlyphArmPosition = robot.motorLift.getCurrentPosition();
                    robot.motorLift.setPower(-.2);
                }
                robot.motorLift.setPower(0);

//                currentSlidePosition = robot.motorLinearSlide.getCurrentPosition();
//                if (currentSlidePosition>5) {
                    robot.motorLinearSlide.setPower(-.6);
//                } else{
//                    robot.motorLinearSlide.setPower(0);
//                }
            } else if (gamepad2.y){            // if gamepad1.y
                currentSlidePosition = robot.motorLinearSlide.getCurrentPosition();
                if (currentSlidePosition<100) {
                    robot.motorLinearSlide.setPower(.6);
                } else{
                    robot.motorLinearSlide.setPower(0);
                }
            }
            else {
                robot.motorLinearSlide.setPower(0);
            }                   // if gamepad2.y

            if (gamepad2.left_stick_y < 0 ) {         // if gamepad2.left_stick_y
                robot.motorLift.setPower(.2);
            } else if (gamepad2.left_stick_y > 0) {
                    robot.motorLift.setPower(-.2);
            } else {
                currentGlyphArmPosition = robot.motorLift.getCurrentPosition();
                if (currentGlyphArmPosition < MINARMPOSITION) {
                    robot.motorLift.setPower(.1);
                }else{
                    robot.motorLift.setPower(0);
                }
              }  // if gamepad2.left_stick_y

            if (gamepad2.right_stick_y != 0) {         // if gamepad2.right_stick_y
                triggerPower = gamepad2.right_stick_y * 0.10;
                robot.motorRelicArm.setPower(triggerPower);
            } else {
                    robot.motorRelicArm.setPower(0);
            }  // if gamepad2.right_stick_y

            if(gamepad2.right_trigger > 0 ){
                robot.servoRelicGrab.setPosition(0);
            }

            if (gamepad2.left_trigger >0) {
                robot.servoRelicGrab.setPosition(.7);
            }
/**            if(gamepad1.x || gamepad2.x) {
                currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();

                if (!relicSet) {          // if !relicSet
                    relicSet = true;
                    setRelicArm();
                } else if (currentLauncherPosition < RELICSETPOSITION){
                    setRelicArm();
                }
            }       // if gamepad2.x
**/

            telemetry.addData("Left_Stick_Y= ", gamepad2.left_stick_y);
            telemetry.addData("Trigger Power = ", triggerPower);
            telemetry.addData("Current Arm Position = ", currentGlyphArmPosition);
            telemetry.addData("Current Slide Position = ", currentSlidePosition);
            telemetry.addData("vlf = ", vlf);
            telemetry.addData("vlr = ", vlr);
            telemetry.addData("vrf = ", vrf);
            telemetry.addData("vrr = ", vrr);
            telemetry.addData("left_stick_x", String.valueOf(gamepad1.left_stick_x));
            telemetry.addData("left_stick_y", String.valueOf(gamepad1.left_stick_y));
            telemetry.addData("right_stick_x", String.valueOf(gamepad1.right_stick_x));
            telemetry.addData("right_stick_y", String.valueOf(gamepad1.right_stick_y));
            telemetry.update();

            idle();
            } // while opModeIsActive

        }


    public void motorsHalt() {              // public void motorsHalt
        robot.motorLF.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
    }                   // public void motorsHalt

    private void begin() {                  // private void begin

        /**
         * Inititialize the robot's hardware.  The hardware configuration can be found in the
         * HardwareTestPlatform.java class.
         */
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }               // private void begin

    private void extendRelicArm(){              //private void extendRelicArm
        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
        targetLauncherPosition = currentLauncherPosition + 200;

        // Lift relic grabber arm up to launching position
        robot.motorRelicArm.setPower(.2);

        while(currentLauncherPosition < targetLauncherPosition) {
            currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();

        }
        robot.motorRelicArm.setPower(0);            // stop the robot relic arm
        // relic arm should be standing straight up

        launchPosition = robot.motorRelicArm.getCurrentPosition();

        /***
         * Control the speed tht the relic arm falls and capture the relic
         */

        robot.motorRelicArm.setPower(.35);
        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
        targetLauncherPosition = currentLauncherPosition + 40;
        while (targetLauncherPosition > currentLauncherPosition){
            currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
        }
        robot.motorRelicArm.setPower(-0.15);

        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
        targetLauncherPosition = currentLauncherPosition + 40;
        while (targetLauncherPosition > currentLauncherPosition){
            currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
        }

        robot.motorRelicArm.setPower(-.05);

    }           //private void extendRelicArm

    private void setRelicArm(){

        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
        targetLauncherPosition = RELICSETPOSITION;

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

        telemetry.addData("current Launcher Position = ", currentLauncherPosition);
        telemetry.addData("target Launcher Position = ", targetLauncherPosition);
        telemetry.addData("Launch Position = ", launchPosition);
        telemetry.addData("Launcher = ", "set!");
        telemetry.update();

    }


    private void retractRelicArm() {                // private void retractRelicArm
        robot.motorRelicArm.setPower(-.5);

        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
        robot.motorRelicArm.setPower(-.1);

        while (launchPosition < currentLauncherPosition){
            currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
        }

        robot.motorRelicArm.setPower(0);
    }                       // private void retractRelicArm
}
