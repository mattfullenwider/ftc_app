/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team7234;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team7234.HardwareBotman;

@TeleOp(name="BotmanTeleOp", group="Pushbot")
//@Disabled
public class BotmanTeleOp extends OpMode{

    /* Declare OpMode members. */
    HardwareBotman robot       = new HardwareBotman();

    //Declares the power scaling of the robot
    private static final double driveCurve = 1.0;

    private boolean isMecanum;
    private boolean bumperToggle;
    private boolean slowDownToggle;

    @Override
    public void init() {

        robot.init(hardwareMap);
        isMecanum = true;
        bumperToggle = true;
        slowDownToggle = true;
    }


    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //TODO: decide on an optimal control scheme and controller count
        //calculates angle in radians based on joystick position, reports in range [-Pi/2, 3Pi/2]
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (Math.PI / 2);
        if (Double.isNaN(angle)){
            angle = 0;              //Prevents NaN error later in the Program
        }

        //calculates robot speed from the joystick's distance from the center
        double magnitude = Math.pow(Range.clip(Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2)), 0, 1), driveCurve);

        // How much the robot should turn while moving in that direction
        double rotation = Range.clip(gamepad1.right_stick_x, -1, 1);

        //Variables for tank drive
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;
        //double armStick = -gamepad2.left_stick_y;
        double armStick = gamepad1.left_trigger - gamepad1.right_trigger;

        robot.arm.setPower(armStick);

        if (bumperToggle){ //Toggles drive mode based on the x button
            if (gamepad1.x){
                isMecanum = !isMecanum;
                bumperToggle = false;
            }
        }
        else if (!gamepad1.x) {
            bumperToggle = true;
        }

/*
        if (gamepad1.y){ //Allows us to toggle speed
            if (slowDownToggle){
                slowDownToggle = false;
                magnitude = magnitude * 0.75;
            }
        }
        if (!gamepad1.y){
            magnitude = Math.pow(Range.clip(Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2)), 0, 1), driveCurve);
            slowDownToggle = true;
        }

        */
        //TODO: Make this toggle work || current issue is in logic of having to press the y button to keep the motors going slow
        if (isMecanum){
            robot.MecanumDrive(angle, magnitude, rotation); //Drives With mecanum
        }

        else{
            if(!gamepad1.right_bumper && !gamepad1.left_bumper){ //Drives as tank
                robot.arrayDrive(left, left, right, right);
            }
            else if (gamepad1.right_bumper && !gamepad1.left_bumper) {  //Strafe right
                robot.arrayDrive(-1, 1, -1, 1);
            }
            else if (!gamepad1.right_bumper) { //Strafe Left
                robot.arrayDrive(1, -1, 1, -1);
            }
            else{
                robot.arrayDrive(0, 0, 0, 0); //Stop
            }
        }

        //Controls Gripper
        if (gamepad1.a){
            robot.gripperOpen();
        }
        else if (gamepad1.b){
            robot.gripperClose();
        }

        telemetry.addData("isMecanum: ", isMecanum);
        telemetry.addLine();
        telemetry.addData("Angle: ", angle);
        telemetry.addData("Magnitude: ", magnitude);
        telemetry.addData("Rotation: ", rotation);
        telemetry.addLine();
        telemetry.addData("X: ", gamepad1.left_stick_x);
        telemetry.addData("Y: ", gamepad1.left_stick_y);
        telemetry.addLine();
        telemetry.addData("FL: ", robot.SpeedsList[0]);
        telemetry.addData("FR: ", robot.SpeedsList[1]);
        telemetry.addData("BR: ", robot.SpeedsList[2]);
        telemetry.addData("BL: ", robot.SpeedsList[3]);


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
