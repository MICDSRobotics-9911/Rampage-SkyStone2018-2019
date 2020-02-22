package org.firstinspires.ftc.teamcode.lib;

public interface TeleOpConstants {

    /**
     * Grabber constants
     * closed = touching a block
     * open = ready for a block
     */
    double GRABBER_PARTIAL_OPEN = 0.25;
    double GRABBER_FULL_OPEN = 1;
    double GRABBER_CLOSED = 0.1; // 0.15
    // double GRABBER_CLOSED_TELEOP_OFFSET = GRABBER_CLOSED + 0.15;

    /**
     * Assist constants
     * closed = touching a block
     * open = ready for a block
     */
    double ASSIST_CLOSED = 0;
    double ASSIST_CLOSED_TELEOP_OFFSET = ASSIST_CLOSED + 0.15;
    double ASSIST_OPEN = 0.75; // 0.65
}
