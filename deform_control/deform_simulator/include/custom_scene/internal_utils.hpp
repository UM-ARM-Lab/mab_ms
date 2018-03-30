#ifndef INTERNAL_UTILS_HPP
#define INTERNAL_UTILS_HPP

#include <stdlib.h>

/**
 * @brief BoxMuller
 * @param m mean
 * @param s standard deviation
 * @return
 */
inline double BoxMuller(double m, double s)
{
    double x1, x2, w, y1;
    static double y2;
    static bool use_last = false;

    /* use value from previous call */
    if (use_last)
    {
        y1 = y2;
        use_last = 0;
    }
    else
    {
        do
        {
            x1 = 2.0 * (double)rand() / (double)RAND_MAX - 1.0;
            x2 = 2.0 * (double)rand() / (double)RAND_MAX - 1.0;
            w = x1 * x1 + x2 * x2;
        }
        while (w >= 1.0);

        w = sqrt((-2.0 * log(w)) / w);
        y1 = x1 * w;
        y2 = x2 * w;
        use_last = true;
    }

    return(m + y1 * s);
}

#endif // INTERNAL_UTILS_HPP
