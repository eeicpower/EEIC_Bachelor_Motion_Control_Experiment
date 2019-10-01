#ifndef _CTRL_IDENT_h
#define _CTRL_IDENT_h



/*	LINEAR CHIRP SIGNAL
**	-------------------------
**	DES:	linear chirp signal for identification
**	INP:	tchirp	: time for chirp (less than chirpTime)
**	OUT:	out   	: calculated current reference
*/
void ctrl_ident_chirplin(double tchirp, double *out, const double f0, const double f1, const double chirpTime);

/*	EXPONENTIAL CHIRP SIGNAL
**	-------------------------
**	DES:	exponential chirp signal for identification
**	INP:	tchirp	: time for chirp (less than chirpTime)
**	OUT:	out   	: calculated current reference
*/
void ctrl_ident_chirpexp(double tchirp, double *out, const double f0, const double f1, const double chirpTime);

#endif
