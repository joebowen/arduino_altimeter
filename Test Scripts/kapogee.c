/*
NAME
	kapogee.c - A third order Kalman filter for
	RDAS raw data files.

SYNOPSIS
	kapogee <infile

DESCRIPTION:
	Performs Kalman filtering on standard input
	data using a third order, or constant acceleration,
	propagation model.

	The standard input data is of the form:
		Column 1: Time of the measurement (seconds)
		Column 2: Acceleration Measurement (ignored)
		Column 3: Pressure Measurement (ADC counts)

	All arithmetic is performed using 32 bit floats.
	The standard output data is of the form:

	Liftoff detected at time: <time>
	Apogee detected at time: <time>

AUTHOR
	David Schultz

*/
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MEASUREMENTSIGMA 0.44
#define MODELSIGMA 0.002
#define MEASUREMENTVARIANCE MEASUREMENTSIGMA*MEASUREMENTSIGMA
#define MODELVARIANCE MODELSIGMA*MODELSIGMA
main()
{
    int             liftoff = 0;
    char            buf[512];
    float           time,
                    accel,
                    pressure;
    float           last_time,
                    last_pressure;
    float           est[3] = { 0, 0, 0 };
    float           estp[3] = { 0, 0, 0 };
    float           pest[3][3] = { 0.002, 0, 0,
	0, 0.004, 0,
	0, 0, 0.002
    };
    float           pestp[3][3] = { 0, 0, 0,
	0, 0, 0,
	0, 0, 0
    };
    float           phi[3][3] = { 1, 0, 0,
	0, 1, 0,
	0, 0, 1.0
    };
    float           phit[3][3] = { 1, 0, 0,
	0, 1, 0,
	0, 0, 1.0
    };
    float           gain[3] = { 0.010317, 0.010666, 0.004522 };
    float           dt;
    float           term[3][3];
    /*
     * Initialize 
     */

    /*
     * Skip text at start of file. 
     */
    while (1) {
	if (gets(buf) == NULL) {
	    fprintf(stderr, "No data in file\n");
	    exit(1);
	} else {
	    break;
	}
    }
    sscanf(buf, "%f,%f,%f", &time, &accel, &pressure);
    est[0] = pressure;
    time = time / 1000;
    last_time = time;

    printf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
	   time, pressure, est[0], est[1], est[2], sqrt(pest[0][0]),
	   sqrt(pest[1][1]), sqrt(pest[2][2]), gain[0], gain[1], gain[2]);

    if (gets(buf) == NULL) {
	fprintf(stderr, "No data\n");
	exit(1);
    }
    sscanf(buf, "%f,%f,%f", &time, &accel, &pressure);
    time = time / 1000;
    est[0] = pressure;
    dt = time - last_time;

    printf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
	   time, pressure, est[0], est[1], est[2], sqrt(pest[0][0]),
	   sqrt(pest[1][1]), sqrt(pest[2][2]), gain[0], gain[1], gain[2]);

    last_time = time;
    /*
     * Fill in state transition matrix and its transpose 
     */
    phi[0][1] = dt;
    phi[1][2] = dt;
    phi[0][2] = dt * dt / 2.0;
    phit[1][0] = dt;
    phit[2][1] = dt;
    phit[2][0] = dt * dt / 2.0;
    while (gets(buf) != NULL) {
	sscanf(buf, "%f,%f,%f", &time, &accel, &pressure);
	time = time / 1000;
	if (last_time >= time) {
	    fprintf(stderr, "Time does not increase.\n");
	    exit(1);
	}
	/*
	 * Propagate state 
	 */
	estp[0] =
	    phi[0][0] * est[0] + phi[0][1] * est[1] + phi[0][2] * est[2];
	estp[1] =
	    phi[1][0] * est[0] + phi[1][1] * est[1] + phi[1][2] * est[2];
	estp[2] =
	    phi[2][0] * est[0] + phi[2][1] * est[1] + phi[2][2] * est[2];
	/*
	 * Simplified version (phi is constant) estp[0] = est[0] +
	 * est[1]*dt + est[2]*dt*dt/2.0; estp[1] = est[1] + est[2]*dt;
	 * estp[2] = est[2]; 
	 */
	/*
	 * Propagate state covariance 
	 */

	term[0][0] =
	    phi[0][0] * pest[0][0] + phi[0][1] * pest[1][0] +
	    phi[0][2] * pest[2][0];
	term[0][1] =
	    phi[0][0] * pest[0][1] + phi[0][1] * pest[1][1] +
	    phi[0][2] * pest[2][1];
	term[0][2] =
	    phi[0][0] * pest[0][2] + phi[0][1] * pest[1][2] +
	    phi[0][2] * pest[2][2];
	term[1][0] =
	    phi[1][0] * pest[0][0] + phi[1][1] * pest[1][0] +
	    phi[1][2] * pest[2][0];
	term[1][1] =
	    phi[1][0] * pest[0][1] + phi[1][1] * pest[1][1] +
	    phi[1][2] * pest[2][1];
	term[1][2] =
	    phi[1][0] * pest[0][2] + phi[1][1] * pest[1][2] +
	    phi[1][2] * pest[2][2];
	term[2][0] =
	    phi[2][0] * pest[0][0] + phi[2][1] * pest[1][0] +
	    phi[2][2] * pest[2][0];
	term[2][1] =
	    phi[2][0] * pest[0][1] + phi[2][1] * pest[1][1] +
	    phi[2][2] * pest[2][1];
	term[2][2] =
	    phi[2][0] * pest[0][2] + phi[2][1] * pest[1][2] +
	    phi[2][2] * pest[2][2];
	pestp[0][0] =
	    term[0][0] * phit[0][0] + term[0][1] * phit[1][0] +
	    term[0][2] * phit[2][0];
	pestp[0][1] =
	    term[0][0] * phit[0][1] + term[0][1] * phit[1][1] +
	    term[0][2] * phit[2][1];
	pestp[0][2] =
	    term[0][0] * phit[0][2] + term[0][1] * phit[1][2] +
	    term[0][2] * phit[2][2];
	pestp[1][0] =
	    term[1][0] * phit[0][0] + term[1][1] * phit[1][0] +
	    term[1][2] * phit[2][0];
	pestp[1][1] =
	    term[1][0] * phit[0][1] + term[1][1] * phit[1][1] +
	    term[1][2] * phit[2][1];
	pestp[1][2] =
	    term[1][0] * phit[0][2] + term[1][1] * phit[1][2] +
	    term[1][2] * phit[2][2];
	pestp[2][0] =
	    term[2][0] * phit[0][0] + term[2][1] * phit[1][0] +
	    term[2][2] * phit[2][0];
	pestp[2][1] =
	    term[2][0] * phit[0][1] + term[2][1] * phit[1][1] +
	    term[2][2] * phit[2][1];
	pestp[2][2] =
	    term[2][0] * phit[0][2] + term[2][1] * phit[1][2] +
	    term[2][2] * phit[2][2];
	pestp[0][0] = pestp[0][0] + MODELVARIANCE;
	/*
	 * Calculate Kalman Gain 
	 */
	gain[0] = (phi[0][0] * pestp[0][0] +
		   phi[0][1] * pestp[1][0] +
		   phi[0][2] * pestp[2][0]) / (pestp[0][0] +
					       MEASUREMENTVARIANCE);
	gain[1] =
	    (phi[1][0] * pestp[0][0] + phi[1][1] * pestp[1][0] +
	     phi[1][2] * pestp[2][0]) / (pestp[0][0] +
					 MEASUREMENTVARIANCE);
	gain[2] =
	    (phi[2][0] * pestp[0][0] + phi[2][1] * pestp[1][0] +
	     phi[2][2] * pestp[2][0]) / (pestp[0][0] +
					 MEASUREMENTVARIANCE);

	/*
	 * Update state and state covariance 
	 */
	est[0] = estp[0] + gain[0] * (pressure - estp[0]);
	est[1] = estp[1] + gain[1] * (pressure - estp[0]);
	est[2] = estp[2] + gain[2] * (pressure - estp[0]);
	pest[0][0] = pestp[0][0] * (1.0 - gain[0]);
	pest[0][1] = pestp[1][0] * (1.0 - gain[0]);
	pest[0][2] = pestp[2][0] * (1.0 - gain[0]);
	pest[1][0] = pestp[0][1] - gain[1] * pestp[0][0];
	pest[1][1] = pestp[1][1] - gain[1] * pestp[1][0];
	pest[1][2] = pestp[2][1] - gain[1] * pestp[2][0];
	pest[2][0] = pestp[0][2] - gain[2] * pestp[0][0];
	pest[2][1] = pestp[1][2] - gain[2] * pestp[1][0];
	pest[2][2] = pestp[2][2] - gain[2] * pestp[2][0];
	/*
	 * Output 
	 */


	printf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
	       time, pressure, est[0], est[1], est[2], sqrt(pest[0][0]),
	       sqrt(pest[1][1]), sqrt(pest[2][2]), gain[0], gain[1],
	       gain[2]);

	if (liftoff == 0) {
	    if (est[1] < -5.0) {
		liftoff = 1;
		printf("Liftoff detected at time: %f\n", time);
	    }
	} else {
	    if (est[1] > 0) {
		printf("Apogee detected at time: %f\n", time);
		exit(0);
	    }
	}

	last_time = time;
    }
}
