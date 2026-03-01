package org.firstinspires.ftc.teamcode;

/**
 * Static storage that survives between Auto and TeleOp within the same
 * app session (same pattern as PoseStorage).
 *
 * motifPattern holds the ball-color order defined by the detected motif tag:
 *   index 0 = first ball to shoot, index 1 = second, index 2 = third
 *   value 1  = Green
 *   value 0  = Purple
 *
 * Tag 21 → Green, Purple, Purple  → {1, 0, 0}
 * Tag 22 → Purple, Green, Purple  → {0, 1, 0}
 * Tag 23 → Purple, Purple, Green  → {0, 0, 1}
 *
 * motifDetected is set to true the first time a motif tag is locked in,
 * so TeleOp knows whether it can trust the pattern.
 */
public class MotifStorage {
    public static int[]   motifPattern  = new int[]{-1, -1, -1}; // -1 = unknown
    public static boolean motifDetected = false;
    public static int     motifTagId    = -1; // which tag was seen (21, 22, or 23)
}