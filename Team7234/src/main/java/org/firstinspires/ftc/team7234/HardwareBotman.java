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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.ExpansionHubMotorControllerPositionParams;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareBotman
{
    /* Public OpMode members. */
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  rightFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor  arm     = null;
    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;
    public Servo jewelPusher = null;
    public ColorSensor jewelColorSensor = null;
    public float hsvValues[] = {0F, 0F, 0F};

    public static final double MID_SERVO       =  0.5 ;
    public static final double RIGHT_GRIPPER_OPEN    =  1 ;
    public static final double LEFT_GRIPPER_OPEN  = 0 ;
    public static final double RIGHT_GRIPPER_CLOSED    =  0 ;
    public static final double LEFT_GRIPPER_CLOSED  = 1;
    public static final double JEWEL_PUSHER_UP = 0.3; //TODO: Find Jewel Pusher Values
    public static final double JEWEL_PUSHER_DOWN = 1.0;

    //Establishes variables for motors
    double[] mecanumSpeeds = {0.0, 0.0, 0.0, 0.0};
    DcMotor[] driveMotors;

    /* local OpMode members. */
    private HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    HardwareBotman(){}
    //region Hardware Map
    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "left Front Drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right Front Drive");
        leftBackDrive = hwMap.get(DcMotor.class, "left Back Drive");
        rightBackDrive = hwMap.get(DcMotor.class, "right Back Drive");
        arm    = hwMap.get(DcMotor.class, "arm");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        arm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetEncoders();

        // Define and initialize ALL installed servos.
        leftClaw  = hwMap.get(Servo.class, "leftClaw");
        rightClaw = hwMap.get(Servo.class, "rightClaw");
        jewelPusher = hwMap.get(Servo.class, "jewelPusher");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);
        jewelPusher.setPosition(JEWEL_PUSHER_UP);
        jewelColorSensor = hwMap.get(ColorSensor.class, "jewelColorSensor");

        driveMotors  = new DcMotor[] {leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive};
    }
    //endregion
    //region Gripper Control

    void gripperOpen() {
        leftClaw.setPosition(LEFT_GRIPPER_OPEN);
        rightClaw.setPosition(RIGHT_GRIPPER_OPEN);
    }
    void gripperClose() {
        leftClaw.setPosition(LEFT_GRIPPER_CLOSED);
        rightClaw.setPosition(RIGHT_GRIPPER_CLOSED);
    }
    //endregion
    //region Robot Driving
    void arrayDrive(double lf, double rf, double lb, double rb){
        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
    }
    void MecanumDrive(double angle, double magnitude, double rotation){  //Calculates and sends values to wheels
        //region Exceptions
        if(angle> 1.5 *Math.PI || angle< -0.5*Math.PI){
            throw new IllegalArgumentException("Angle is outside range [-pi/2, 3pi/2]. Invalid Value is: " + Double.toString(angle));
        }

        if(magnitude<0 || magnitude>1){
            throw new IllegalArgumentException("Magnitude is outside range [0, 1]. Invalid Value is: " + Double.toString(magnitude));
        }
        if(rotation<-1 || rotation>1){
            throw new IllegalArgumentException("Rotation is outside range [-1, 1]. Invalid Value is: " + Double.toString(rotation));
        }
        //endregion
        //region Initial Speeds
        mecanumSpeeds[0] = ((magnitude*(Math.sin(angle+(Math.PI/4))))+rotation);
        mecanumSpeeds[1] = -((magnitude*(Math.cos(angle+(Math.PI/4))))-rotation);  //Generates Raw Values for Motors
        mecanumSpeeds[2] = ((magnitude*(Math.cos(angle+(Math.PI/4))))+rotation);
        mecanumSpeeds[3] = -((magnitude*(Math.sin(angle+(Math.PI/4))))-rotation);
        //endregion
        //region Speed Divider
        double speedDivider = Math.abs(mecanumSpeeds[0]);
        //Speed divider is set as
        for (int i=0; i<4; i++){
            if (Math.abs(mecanumSpeeds[i]) > speedDivider){
                speedDivider = Math.abs(mecanumSpeeds[i]);
            }
        }

        if (speedDivider > 1) {            //SpeedDivider is only called if it is necessary to maintain ranges
            for (int i=0; i<4; i++) {
                mecanumSpeeds[i] /= speedDivider;
            }
        }
        //endregion
        //region Power Assignment
        for (int i =0; i<4; i++){
            mecanumSpeeds[i] = Range.clip(mecanumSpeeds[i], -1.0, 1.0);
            driveMotors[i].setPower(mecanumSpeeds[i]);
        }
        //endregion
    }
    //endregion
    //Function to limit values to a range
    private double clip(double input, double min, double max){   //Method for clipping a value within a range
        double output = input;
        if (input < min){
            output = min;
        }
        else if (input > max){
            output = max;
        }
        return output;
    }

    public double ticsPerInch(double distance){
        return (280/Math.PI) * distance;
    }

    public void resetEncoders() {
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

 }

