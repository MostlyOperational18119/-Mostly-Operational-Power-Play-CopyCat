package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.motorSlide;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TimerTeleOp")
@Disabled
public class TimerTeleOp extends DriveMethods {
    @Override
    public void runOpMode() {
        telemetry.addLine("Init");
        telemetry.update();
//        initMotorsBlue();
//        motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        Timer theTimer = new Timer();
        theTimer.startTimer();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                theTimer.startTimer();
            }
            if(gamepad1.b) {
                theTimer.stopTimer();
            }
            if (gamepad1.x) {
                theTimer.resetTimer();
            }
            telemetry.addData("Current Time: ", theTimer.getTime());
            telemetry.update();
        }
    }
}
