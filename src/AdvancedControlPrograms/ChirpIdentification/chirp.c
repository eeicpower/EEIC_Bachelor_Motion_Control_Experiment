//inert.c
#include <math.h>
#include "chirp.h"
//#include "system_math.h"

//#define	f0	0.1		// initial frequency [Hz]
//#define f1	50	// final frequency [Hz]
//#define	chirpTime	2	// charp signal period [s]
#define PI (3.14159265358979323)



void ctrl_ident_chirplin (double tchirp, double *out, const double f0, const double f1, const double chirpTime)
{
	double	klin = (f1 - f0) / chirpTime;	// rate of frequency change	
	while (tchirp > chirpTime) {
		tchirp -= chirpTime;
	}
	
	*out = sin(PI*2.0*(f0 * tchirp + klin * tchirp * tchirp / 2.0));
}


void ctrl_ident_chirpexp(double tchirp, double *out, const double f0, const double f1, const double chirpTime)
{
	double	klin = (f1 - f0) / chirpTime;	// rate of frequency change
	double	kexp;
	kexp = pow((f1 / f0), (1 / chirpTime)); // rate of exponential change in frequency

	while (tchirp > chirpTime) {
		tchirp -= chirpTime;
	}
	*out = sin(PI*2.0*f0*((pow(kexp, tchirp) - 1) / log(kexp)));
}



