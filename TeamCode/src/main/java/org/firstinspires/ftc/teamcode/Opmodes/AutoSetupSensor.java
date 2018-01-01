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

/**
 * Import the classes we need to have local access to.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;
import org.firstinspires.ftc.teamcode.Libs.VuforiaLib;

import java.util.List;
import java.util.Locale;

/**
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "SETUP - Check Sensors", group = "SETUP")

/**
 * This opMode attempts to acquire and trigger the rightRed beacons.
 *
 * Assumptions:
 *
 *  - The robot starts facing the positive Y axis (facing the the rightBlue wall
 *  with the rightBlue images, legos and tools)
 *  -  The robot starts with the left side wheels just to the left of the seam between tiles 2 and 3
 *  on the rightRed team wall with tile one being the left most tile on the wall, ~ -1500 X from origin
 */
public class AutoSetupSensor extends LinearOpMode {

    /**
     * Instantiate all objects needed in this class
     */
    public double colorRightRed = 0;
    public double colorRightBlue = 0;
    public double colorLeftRed = 0;
    public double colorLeftBlue = 0;
    public double currentZint = 0;
    public double rangeDistance = 0;
    public double currentLauncherPosition =0;
    public double currentGlyphArmPosition = 0;

    private final static HardwareTestPlatform robot = new HardwareTestPlatform();

    /**
     * Setup the init state of the robot.  The actions in the begin() method are run when you hit the
     * init button on the driver station app.
     */
    private void begin() {

        /**
         * Inititialize the robot's hardware.  The hardware configuration can be found in the
         * HardwareTestPlatform.java class.
         */
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void runOpMode() {
        begin();


        /**
         * Start the opMode
         */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            getSensorData();

            telemetry.addData("range = ", rangeDistance);
            telemetry.addData("gyro = ", currentZint);
            telemetry.addData("colorLeftRed = ", colorLeftRed);
            telemetry.addData("colorLeftBlue = ", colorLeftBlue);
            telemetry.addData("colorRightRed = ", colorRightRed);
            telemetry.addData("colorRightBlue = ", colorRightBlue);
            telemetry.addData("Current Launcher Position = ", currentLauncherPosition);
            telemetry.addData("Current Glyph Arm Position = ", currentGlyphArmPosition);
            telemetry.update();

            idle();

        }

        //Exit the OpMode
        requestOpModeStop();
    }


    private void getSensorData() {
        colorRightRed = robot.colorSensorRight.red();
        colorRightBlue = robot.colorSensorRight.blue();
        colorLeftRed = robot.colorSensorLeft.red();
        colorLeftBlue = robot.colorSensorLeft.blue();
        currentZint = robot.mrGyro.getIntegratedZValue();
        rangeDistance = robot.rangeSensor.cmUltrasonic();
        currentLauncherPosition = robot.motorRelicArm.getCurrentPosition();
        currentGlyphArmPosition = robot.motorLift.getCurrentPosition();

    }


}
