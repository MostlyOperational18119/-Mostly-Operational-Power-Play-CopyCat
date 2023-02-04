package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Timer {
    ElapsedTime daTime = new ElapsedTime();
    Boolean timerStopped = true;
    double cTime = 0;
    public void startTimer() {
        daTime.startTime();
        timerStopped = true;
    }
    public void resetTimer() {
        daTime.reset();
    }
    public void stopTimer() {
        cTime = daTime.seconds();
        timerStopped = true;
    }
    public double getTime() {
        if (!timerStopped) {
            cTime = daTime.seconds();
        }
        return cTime;
    }
    public void cleanup() {
        timerStopped =  null;
        daTime = null;
        stopTimer();
    }
}
