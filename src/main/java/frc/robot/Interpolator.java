/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class Interpolator {
    private static double angles[] = {14.02984, 13.06595, 10.573, 8.05872, 7.395187, 6.2501, 5.351089, 4.949104, 4.49338, 3.239723, 2.73799, 2.420449, 1.858061, 1.541063, 1.481606};
    private static double speeds[] = {13500, 13000, 13000, 13000, 12700, 12700, 12700, 12700, 12800, 13450, 14000, 15000, 16000, 17600, 18000};
    private static double m_offset = 0;

    public static double getInterpolation(double angle) {
        int i = 0;
        while(angles[i] > angle) {
            i++;
            if(i >= angles.length) {
                return 0;
            }
        }
        if(i == 0) {
            return speeds[i];
        }

        int i1 = i-1;
        int i2 = i;
        double m = (speeds[i2]-speeds[i1])/(angles[i2]-angles[i1]);
        double b = speeds[i1] - m*angles[i1];
        double interpolation = m*angle+b + 100 + m_offset;
        return interpolation;
    }

    public static void setOffset(double offset) {
        m_offset = offset;
    }
}
