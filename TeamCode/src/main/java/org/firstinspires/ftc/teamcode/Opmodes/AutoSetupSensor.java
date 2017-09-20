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
    public static final String TAG = "VuforiaLib Sample";
    private final static HardwareTestPlatform robot = new HardwareTestPlatform();
    private ElapsedTime runtime = new ElapsedTime();
    // the the current vuforia image
    private List<Double> vuforiaTracking;
    private VuforiaLib myVuforia = new VuforiaLib();
    private DataLogger Dl;
    Orientation angles;
    private double initZ = 0;

    /**
     * Setup the init state of the robot.  The actions in the begin() method are run when you hit the
     * init button on the driver station app.
     */
    private void begin() {

        /** Setup the dataLogger
         * The dataLogger takes a set of fields defined here and sets up the file on the Android device
         * to save them to.  We then log data later throughout the class.
         */
        Dl = new DataLogger("Sensor_Check" + runtime.time());
        Dl.addField("touchSensor");
        Dl.addField("robotX");
        Dl.addField("robotY");
        Dl.addField("robotBearing");
        Dl.addField("zInt");
        Dl.addField("ODS");
        Dl.addField("rgb");
        Dl.addField("rightRed");
        Dl.addField("rightBlue");
        Dl.addField("leftRed");
        Dl.addField("leftBlue");
        Dl.addField("beacon color");
        Dl.addField("LRMotorPos");
        Dl.newLine();

        /**
         * Inititialize the robot's hardware.  The hardware configuration can be found in the
         * HardwareTestPlatform.java class.
         */
        robot.init(hardwareMap);

        /**
         * Calibrate the gyro
         */

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void runOpMode() {
        begin();
        List<VuforiaTrackable> myTrackables;
        myTrackables = myVuforia.vuforiaInit();

        /**
         * Start the opMode
         */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            vuforiaTracking = myVuforia.getLocation(myTrackables);
            angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            initZ = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

            logData();

            idle();

        }
        //Stop the motors
        motorsHalt();

        //Stop the DataLogger
        dlStop();

        //Exit the OpMode
        requestOpModeStop();
    }


    private void logData() {
        telemetry.addData("Elapsed Time", String.valueOf(runtime.milliseconds()));
        telemetry.addData("initZ", String.valueOf(initZ));


        telemetry.update();

        Dl.addField(String.valueOf(robot.touchSensor.getValue()));

        Dl.newLine();
    }

    /**
     * Stop the DataLogger
     */
    private void dlStop() {
        Dl.closeDataLogger();

    }

    /**
     * Cut power to the motors.
     */
    private void motorsHalt() {
        robot.motorLF.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
