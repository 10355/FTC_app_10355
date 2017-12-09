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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;
import org.firstinspires.ftc.teamcode.Libs.VuforiaLib;

/**
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "foo", group = "COMP")
@Disabled

public class foo extends LinearOpMode {

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


    public double LF, RF, LR, RR;
    public double currentZint = 0;
    private long z = 0;
    private boolean cts = true;
    public double zCorrection = 0;


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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(1000);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        /**
         * Deploy the color sensor
         */

        /*** initilize servos ***/
        robot.servoLeft.setPosition(.5);
        robot.servoRight.setPosition(.5);
        sleep(2000);

        telemetry.addData(">", "Servos");
        telemetry.update();

        /**
         * Start the opMode
         */
        waitForStart();
        telemetry.addData(">", "started");
        telemetry.update();



        while (opModeIsActive()) {
            while (cts == true) {
                /**
                 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                 * This is the section of code you should change for your robot.
                 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                 */


                switch (state) {
                    case DRIVE:
                        drive.translateTime(10, 1, 0);

                        telemetry.addData(">", "drive.translateTime");
                        telemetry.update();

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
        }}


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
        HALT, DRIVE,
    }

}
