package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RotationTeleOp",group="A")

public class RotationTeleOp extends DriveMethods {
    @Override
    public void runOpMode() {
        initMotorsBlue();
        calibrateNavXIMU();
        int rotationZ = 0;
        waitForStart();
        while(opModeIsActive()) {
            rotationZ += Math.floor(gamepad1.right_stick_x * 5);
            if(gamepad1.a) {
                rotateToHeading(rotationZ,0.2);
            }
            sleep(100);
        }
    }
}
