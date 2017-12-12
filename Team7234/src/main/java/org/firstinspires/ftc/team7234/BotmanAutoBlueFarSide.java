/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.team7234;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static com.sun.tools.javac.util.Constants.format;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Botman Auto Blue Far", group = "Example")
//@Disabled
public class BotmanAutoBlueFarSide extends OpMode {

    RelicVuMarkIdentification2 relicVuMark = new RelicVuMarkIdentification2();
    public RelicRecoveryVuMark keyFinder;
    HardwareBotman robot = new HardwareBotman();


    currentState programState = currentState.KEY;
    public enum currentState {
        KEY,
        JEWELS,
        MOVE,
        LEFT, CENTER, RIGHT,
        SCORE
    }
//Swag 420 blaze it
    @Override
    public void init() {
        robot.init(hardwareMap);
        relicVuMark.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() { }


    @Override
    public void start() {
        relicVuMark.start();

    }


    @Override
    public void loop() {
        keyFinder = relicVuMark.readKey();
        if (relicVuMark.vuMark != RelicRecoveryVuMark.UNKNOWN) {

            telemetry.addData("VuMark", "%s visible", keyFinder);
        } else {
            telemetry.addData("VuMark", "not visible");
        }
        relicVuMark.vuMark = RelicRecoveryVuMark.from(relicVuMark.relicTemplate);
        switch (programState) {

            case KEY:

                telemetry.addData("We are seeing", keyFinder);
                programState = currentState.JEWELS;
                break;

            case JEWELS:
                Color.RGBToHSV(robot.jewelColorSensor.red() * 8, robot.jewelColorSensor.green() * 8, robot.jewelColorSensor.blue() * 8, robot.hsvValues);
                robot.jewelPusher.setPosition(.1);

                if(robot.hsvValues[0] > 210 || robot.hsvValues[0] < 240){
                    if(robot.leftBackDrive.getCurrentPosition() <= robot.ticsPerInch(1)){
                        robot.arrayDrive(0.5, 0, 0.5, 0);
                    }
                    else if (robot.leftBackDrive.getCurrentPosition() >= robot.ticsPerInch(-0.9)){
                        robot.jewelPusher.setPosition(.9);
                        robot.arrayDrive(-0.5, 0, -0.5, 0);
                        programState = currentState.MOVE;
                    }
                }
                else if(robot.hsvValues[0] > 345 || robot.hsvValues[0] < 15) {
                    if(robot.rightBackDrive.getCurrentPosition() <= robot.ticsPerInch(1)){
                        robot.arrayDrive(0, 0.5, 0, 0.5);
                    }
                    else if (robot.rightBackDrive.getCurrentPosition() >= robot.ticsPerInch(-0.9)){
                        robot.jewelPusher.setPosition(.9);
                        robot.arrayDrive(0, -0.5, 0, -0.5);
                        programState = currentState.MOVE;
                    }
                }
                telemetry.addData("HSV is", robot.hsvValues);
                break;

            /*case MOVE:
                robot.arrayDrive(1, 1, 1, 1);

                if (robot.leftBackDrive.getCurrentPosition() >= Math.abs(robot.ticsPerInch(12))){
                    robot.MecanumDrive(0, 0, 0);
                }
                else{
                    robot.resetEncoders();
                    if (keyFinder.equals("L")){
                        programState = currentState.LEFT;
                    }
                    else if (keyFinder.equals("C")){
                        programState = currentState.CENTER;
                    }
                    else if (keyFinder.equals("R")){
                        programState = currentState.RIGHT;
                    }
                }
                break;

            /*case LEFT:


            case CENTER:


            case RIGHT:


            case SCORE:
                //Score glyph*/
        }


    }


    @Override
    public void stop() { }
}
