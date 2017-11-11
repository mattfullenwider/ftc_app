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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.team7234.RelicVuMarkIdentification2;
import org.firstinspires.ftc.team7234.HardwareBotman;

import static com.sun.tools.javac.util.Constants.format;
import java.util.Random;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Botman Auto Test", group = "Example")
//@Disabled
public class BotmanAutoSkeleton extends OpMode {

    RelicVuMarkIdentification2 relicVuMark = new RelicVuMarkIdentification2();
    HardwareBotman robot = new HardwareBotman();

    Random LRC = new Random();

    @Override
    public void init() {
        robot.init(hardwareMap);
        relicVuMark.init();
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
        relicVuMark.loop();
        relicVuMark.vuMark = RelicRecoveryVuMark.from(relicVuMark.relicTemplate);
        if (relicVuMark.vuMark != RelicRecoveryVuMark.UNKNOWN) {

            relicVuMark.pose = relicVuMark.relicTemplateListener.getPose();
            if (format(relicVuMark.pose).equals("L")){
                //Do thing for left glyph position
            }
            if (format(relicVuMark.pose).equals("C")){
                //Do thing for center glyph position
            }
            if (format(relicVuMark.pose).equals("R")){
                //Do thing for right glyph position
            }
        }
        //The else statement chooses a random integer and uses a position based on it
        //This is only to look like we know what we are doing
        //If you want, we can just go for the middle if we cannot detect the image, but I like this better.
        else {
            int randInt = LRC.nextInt(2);
            telemetry.addData("Random Integer is %s", randInt);
            if (randInt == 0){
                //Do thing for left glyph position
            }
            if (randInt == 1){
                //Do thing for center glyph position
            }
            if (randInt == 2){
                //Do thing for right glyph position
            }

        }
    }


    @Override
    public void stop() { }
}
