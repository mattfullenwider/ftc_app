/* Copyright (c) 2015 Qualcomm Technologies Inc

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

package org.firstinspires.ftc.team535;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TOBOR Tank Drive", group = "Teleop")
@Disabled

public class TOBORTank extends OpMode {

    DcMotor FRMotor;
    DcMotor FLMotor;
    DcMotor BRMotor;
    DcMotor BLMotor;
    DcMotor rightTrack;
    DcMotor leftTrack;
    Servo RPlate;
    Servo LPlate;

    @Override
    public void init() {
        FRMotor = hardwareMap.dcMotor.get("FRight");
        FLMotor = hardwareMap.dcMotor.get("FLeft");
        BRMotor = hardwareMap.dcMotor.get("BRight");
        BLMotor = hardwareMap.dcMotor.get("BLeft");
        //rightTrack = hardwareMap.dcMotor.get("rTrack");
        //leftTrack = hardwareMap.dcMotor.get("lTrack");


        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        rightTrack.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status:", "Robot is Initialized");
    }

    @Override
    public void init_loop() { }


    @Override
    public void loop() {
        if (gamepad1.right_trigger>=0.1)
        {
            BLMotor.setPower(-1*gamepad1.left_trigger);
            BRMotor.setPower(-1*gamepad1.left_trigger);
            FRMotor.setPower(1*gamepad1.left_trigger);
            FLMotor.setPower(1*gamepad1.left_trigger);
        }
        else if (gamepad1.left_trigger>=0.1)
        {
            BLMotor.setPower(1*gamepad1.left_trigger);
            BRMotor.setPower(1*gamepad1.left_trigger);
            FRMotor.setPower(-1*gamepad1.left_trigger);
            FLMotor.setPower(-1*gamepad1.left_trigger);
        }
        else
        {
            FRMotor.setPower(Range.clip(gamepad1.right_stick_y, -1, 1));
            BRMotor.setPower(Range.clip(gamepad1.right_stick_y, -1, 1));
            FLMotor.setPower(Range.clip(gamepad1.left_stick_y, -1, 1));
            BLMotor.setPower(Range.clip(gamepad1.left_stick_y, -1, 1));
        }

        // send the info back to driver station using telemetry function.
        if (gamepad1.left_bumper)
        {
            rightTrack.setPower(-1);
            leftTrack.setPower(-1);
        }
        if (gamepad1.right_bumper)
        {
            rightTrack.setPower(1);
            leftTrack.setPower(1);
        }

        if (gamepad1.a)
        {
            RPlate.setPosition(RPlate.getPosition()+0.002);
            LPlate.setPosition(LPlate.getPosition()-0.002);
        }
        if (gamepad1.b)
        {
            RPlate.setPosition(RPlate.getPosition()-0.002);
            LPlate.setPosition(LPlate.getPosition()+0.002);
        }
    }


    @Override
    public void stop() {
        FRMotor.setPower(0);
        BRMotor.setPower(0);
        FLMotor.setPower(0);
        BLMotor.setPower(0);
        rightTrack.setPower(0);
        leftTrack.setPower(0);
    }
}