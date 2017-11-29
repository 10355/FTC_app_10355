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
    double rangeDistance=0;
    double targetLauncherPosition = 0;
    double launchPosition = 0;
    boolean relicSet = false;              // flag to identify if the relic arm has been deployed
    boolean relicDeploy = false;             // flag to identify if the relic arm is deployed
    boolean relicCaptured = false;          // flag to identify if the relic is captured

    @Override
    public void runOpMode() {

        begin();

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.servoStone.setPosition(.8);                        // move servoStone out of the way
        robot.servoLeft.setPosition(1);                          // move servoLeft out of the way
        robot.servoRight.setPosition(0);                         // move servoRight out of the way

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Waiting for Teleop to start.", "");    //
        telemetry.update();
        waitForStart();
        telemetry.addData("Teleop Test Active", "");    //
        telemetry.update();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y *-1);
            double robotAngle = Math.atan2((gamepad1.left_stick_y*-1), (gamepad1.left_stick_x )) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            telemetry.addData("Speed Value = ", speed);
            telemetry.addData("Robot Angle", robotAngle);
            telemetry.addData("rightX", rightX);

            final double vlf = speed * Math.cos(robotAngle) + rightX;
            final double vrf = speed * Math.sin(robotAngle) - rightX;
            final double vlr = (speed * Math.sin(robotAngle) + rightX)*.8;
            final double vrr = (speed * Math.cos(robotAngle) - rightX)*.8;

            robot.motorLF.setPower(vlf);
            robot.motorRF.setPower(vrf);
            robot.motorLR.setPower(vlr);
            robot.motorRR.setPower(vrr);

            if (gamepad1.x) {
                robot.servoBlockExit.setPosition(.5);
            } else{
                robot.servoBlockExit.setPosition(1);
            }

            if (gamepad1.left_bumper){
                rangeDistance = robot.rangeSensor.cmUltrasonic();
                while (rangeDistance > 25){
                    robot.motorLF.setPower(-1);
                    robot.motorLR.setPower(-1);
                    robot.motorRF.setPower(-1);
                    robot.motorRR.setPower(-1);
                }
                motorsHalt();
            }

            if (gamepad1.right_bumper){
                rangeDistance = robot.rangeSensor.cmUltrasonic();
                while (rangeDistance < 100){
                    robot.motorLF.setPower(1);
                    robot.motorLR.setPower(1);
                    robot.motorRF.setPower(1);
                    robot.motorRR.setPower(1);
                }
                motorsHalt();


                if (gamepad1.left_bumper){
                rangeDistance = robot.rangeSensor.cmUltrasonic();
                while (rangeDistance > 20){
                    robot.motorLF.setPower(1);
                    robot.motorLR.setPower(1);
                    robot.motorRF.setPower(1);
                    robot.motorRR.setPower(1);
                }
                motorsHalt();

            }

            if (gamepad1.right_trigger >0) {
                robot.servoLiftRight.setPosition(1);
                robot.servoLiftLeft.setPosition(0);
            }  // gamepad1.right_bumper

            if (gamepad1.left_trigger >0) {
                robot.servoLiftRight.setPosition(.75);
                robot.servoLiftLeft.setPosition(.25);
            }  // gamepad1.left_bumper

            if (gamepad1.y){
                robot.motorLinearSlide.setPower(.4);
            } else if (gamepad1.a) {
                robot.motorLinearSlide.setPower(-.4);

            }else {
                robot.motorLinearSlide.setPower(0);
            }

            if (gamepad1.dpad_up) {
                robot.motorLift.setPower(.3);
            }
            else if (gamepad1.dpad_down) {
                robot.motorLift.setPower(-.1);
            }
            else {
                robot.motorLift.setPower(0);
            }  // gamepad2.left_trigger

            if(gamepad2.dpad_down){

                if(!relicSet){
                    relicSet = true;
                    currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
                    targetLauncherPosition = currentLauncherPosition + 500;

                    // Lift relic grabber arm up to launching position
                    robot.motorRelicArm.setPower(.2);

                    while(currentLauncherPosition < targetLauncherPosition) {
                        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();

                    }
                    robot.motorRelicArm.setPower(0);            // stop the robot relic arm
                    // relic arm should be standing straight up

                    launchPosition = robot.motorRelicArm.getCurrentPosition();

                } else if (!relicCaptured){   // else if (!relicCaptured)
                    relicCaptured = true;

                    extendRelicArm();

                    sleep(100);

                    robot.servoRelicGrab.setPosition(.5);

                    robot.motorRelicArm.setPower(-.1);

                    sleep(2000);

                    retractRelicArm();

                } else if (relicCaptured) {   // else if (!relicCaptured)

                    relicCaptured = false;
                    extendRelicArm();

                    sleep(100);

                    robot.servoRelicGrab.setPosition(0);

                    sleep(2000);

                    retractRelicArm();

                }

            }else if(gamepad2.dpad_up){

            }

            if (gamepad1.y){
                robot.servoRelicGrab.setPosition(0);

            }

            if(gamepad2.a == true) {
                robot.servoStone.setPosition(.71);
                sleep(800);
                robot.motorLF.setPower(.75);
                robot.motorRF.setPower(.75);
                robot.motorLR.setPower(.75);
                robot.motorRR.setPower(.75);

                sleep(300);

                robot.servoStone.setPosition(.8);

                sleep(700);

                motorsHalt();
            }  // if gamepad2.a

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

        } // runOpMode

    public void motorsHalt() {
        robot.motorLF.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
    }

    private void begin() {

        /**
         * Inititialize the robot's hardware.  The hardware configuration can be found in the
         * HardwareTestPlatform.java class.
         */
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void extendRelicArm(){
        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
        targetLauncherPosition = currentLauncherPosition + 500;

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

        robot.motorRelicArm.setPower(.3);
        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
        targetLauncherPosition = currentLauncherPosition + 40;
        while (targetLauncherPosition > currentLauncherPosition){
            currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
        }
        robot.motorRelicArm.setPower(.1);

        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
        targetLauncherPosition = currentLauncherPosition + 40;
        while (targetLauncherPosition > currentLauncherPosition){
            currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
        }

        robot.motorRelicArm.setPower(.1);

    }

    private void retractRelicArm() {
        robot.motorRelicArm.setPower(-.3);

        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
        while (launchPosition < currentLauncherPosition){
            currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
        }

        robot.motorRelicArm.setPower(0);

    }
}

}
