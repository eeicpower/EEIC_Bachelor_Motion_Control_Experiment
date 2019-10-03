//pd.c
#include <stdlib.h>
#include <stdio.h>
#include <sys/io.h>
#include <sys/signal.h>
#include <math.h>
#include </usr/include/fbida.h> /* iji */
#include </usr/include/fbiad.h> /* iji */
#include "/usr/src/linux-2.4.20-ART/include/linux/art_types.h" /*  iji */
#include "/usr/src/linux-2.4.20-ART/include/linux/art_task.h" /* iji  */
#define TRUE 1

#define KBD_PORT 0x61
#define SPK_BIT  0x02
#define LP0_PORT 0x378
#define ENCADRES0 0x9800
#define ADADRES0 0x9000
#define DAADRES0 0x9400
#define ENC_RESL 312500

int Ts,Tcon;

/* motor constants */
double Jn; //[kgm^2]
double Ktn = 0.6; //[Nm/A]
	// D/A output:-10[V]~10[V]
	// Virtual torque current:-10[A]~10[A]
	// corresponding torque:6 Nm
	// 					-9.8[Nm]~9.8[Nm]

/* controller variables definition */
double I_ref=0.0, I_limit;
double X, X_ref;
double u_x, u, uxn_1=0.0, iu=0.0, iu_1=0.0, u_1=0.0, du;
double Tr, Zt, Kp, Kd, Ki;
double Tau;
double T_smpl=0.0;
double b0,b1,b2;
double c1, c2, cu;
double y,ddX;
double I01, I01_1=0.0, I0n,I0n_1=0.0;
double x1=0.0,x1n=0.0,x2=0.0,x1n_1=0.0,x2n=0.0,x2n_1=0.0, x2n_2=0.0;
/*----------------------------------------------*/
double control(double X_r,double Xs)
{

/* IO */
	
	u = X_r - Xs; 

	u_x = u - x2n_1/Tau;
	x2n = x2n_1 + (u_x+uxn_1)*T_smpl/2.0;
	du = u/Tau - x2n/Tau/Tau;

	iu = iu_1 + (u+u_1)*T_smpl/2.0;
	
	y=Ki*iu + Kd*du + Kp*u;

	uxn_1 = u_x;
	u_1 = u;
	iu_1 = iu;
	x2n_1=x2n;

/* I_ref */
	return y;
}
/*----------------------------------------------*/
/* encorder initialize */
int enc_init(){
	fprintf(stderr,"Encoder ADRESS: %4x\n",ENCADRES0);
	outb(0x6,ENCADRES0 + 0x04);
	outb(0x6,ENCADRES0 + 0x14);
	return 0;
}
/* Ana->Dig transfer */
static double Adtransfer(int ch)
{
unsigned short SmplAd=0;
unsigned char lmb,hmb;
	switch(ch){
	case 1:
	outb(0x40,ADADRES0);
	while(inb(ADADRES0+0x3)!=0x80){}
	lmb=inb(ADADRES0);
	hmb=inb(ADADRES0+0x1)-0x80;

	SmplAd = hmb*256+lmb;
	break;
	case 2:
	outb(0x41,ADADRES0);
	while(inb(ADADRES0+0x3)!=0x80){}
	lmb=inb(ADADRES0);
	hmb=inb(ADADRES0+0x1)-0x80;
	SmplAd = hmb*256+lmb;
	break;
	case 3:
	outb(0x42,ADADRES0);
	while(inb(ADADRES0+0x3)!=0x80){}
	lmb=inb(ADADRES0);
	hmb=inb(ADADRES0+0x1)-0x80;
	SmplAd = hmb*256+lmb;
	break;
	case 4:
	outb(0x43,ADADRES0);
	while(inb(ADADRES0+0x3)!=0x80){}
	lmb=inb(ADADRES0);
	hmb=inb(ADADRES0+0x1)-0x80;
	SmplAd = hmb*256+lmb;
	break;
	case 5:
	outb(0x44,ADADRES0);
	while(inb(ADADRES0+0x3)!=0x80){}
	lmb=inb(ADADRES0);
	hmb=inb(ADADRES0+0x1)-0x80;
	SmplAd = hmb*256+lmb;
	break;
	case 6:
	outb(0x45,ADADRES0);
	while(inb(ADADRES0+0x3)!=0x80){}
	lmb=inb(ADADRES0);
	hmb=inb(ADADRES0+0x1)-0x80;
	SmplAd = hmb*256+lmb;
	break;
	case 7:
	outb(0x46,ADADRES0);
	while(inb(ADADRES0+0x3)!=0x80){}
	lmb=inb(ADADRES0);
	hmb=inb(ADADRES0+0x1)-0x80;
	SmplAd = hmb*256+lmb;
	break;
	case 8:
	outb(0x47,ADADRES0);
	while(inb(ADADRES0+0x3)!=0x80){}
	lmb=inb(ADADRES0);
	hmb=inb(ADADRES0+0x1)-0x80;
	SmplAd = hmb*256+lmb;
	break;
	}
	return (double)(20.0*SmplAd)/4096.0-10.0;
}
/* Dig->Ana transfer */
static int Datransfer(int ch,double DaVout)
{
	unsigned char lmb,hmb;
	unsigned short SmplDa;
	if(DaVout>=5.0){
	SmplDa = 4095;
	}
	else{
		if(DaVout<=-5.0){
		SmplDa = 0;
		}
		else{
		SmplDa = DaVout*4096.0/10.0+2048;
		}
	}
	switch(ch){
	case 1:
	outb(0x3,DAADRES0 + 0x0b);
	outb(0x0,DAADRES0 + 0x02);
	lmb=SmplDa;
	hmb=SmplDa/256;
	outb(lmb,DAADRES0);
	outb(hmb,DAADRES0 + 0x01);
	break;

	case 2:
	outb(0x3,DAADRES0 + 0x0b);
	outb(0x1,DAADRES0 + 0x02);
	lmb=SmplDa;
	hmb=SmplDa/256;
	outb(lmb,DAADRES0);
	outb(hmb,DAADRES0 + 0x01);	
	break;

	case 3:
	outb(0x4,DAADRES0 + 0x0b);
	outb(0x2,DAADRES0 + 0x02);
	lmb=SmplDa;
	hmb=SmplDa/256;
	outb(lmb,DAADRES0);
	outb(hmb,DAADRES0 + 0x01);	
	break;

	case 4:
	outb(0x8,DAADRES0 + 0x0b);
	outb(0x3,DAADRES0 + 0x02);
	lmb=SmplDa;
	hmb=SmplDa/256;
	outb(lmb,DAADRES0);
	outb(hmb,DAADRES0 + 0x01);	
	break;
	}
	return 0;
}

/* read encorder */
static double read_theta(int ch)
{
	int cnt;
	double rad;
	unsigned char lbyt,m1byt,m2byt,hbyt;
	unsigned int adr;

	switch(ch){
	case 1:
	adr = ENCADRES0;
	break;
	case 2:
	adr = ENCADRES0 + 0x10;
	break;
	default:
	adr=0;
	break;
	}

	outb(0x2,ENCADRES0 + 0x06);
	lbyt=inb(adr);
	m1byt=inb(adr+0x1);
	m2byt=inb(adr+0x2);
	hbyt=inb(adr+0x3);
	cnt = hbyt*16777216+m2byt*65536+m1byt*256+lbyt;
	rad=(double)cnt*(-2.0)*M_PI/ENC_RESL;
	return(rad);
}

/* real time stop */
void ctlstop(){
unsigned char 	OutChar;
	OutChar=0;
	outb(OutChar, LP0_PORT);
	fprintf(stderr,"motor-stop\n");
	
	Datransfer(1,0.0);

	if(DaClose(1)==-1||AdClose(1)){
	fprintf(stderr,"iopl failed\n");
	exit(1);
	}

      if (art_exit() == -1) {
          perror("art_exit");
          exit(1);
      }

	return;
}

int main(int argc, char *argv[])
{
int i, cntnum, beep;
unsigned char 	OutChar=0;
double theta1, Data, Vout;
int res,dnum;
unsigned long ulpNum;
ADSMPLREQ Smplreq;
DASMPLREQ Conf;
FILE *resfile;
double *tmpDataI,*tmpDataXref,*tmpDataX;

/* sampling time */
	Ts=0;
	while(Ts == 0){
	printf("\n sampling time [msec] (1msec):");
	scanf("%d",&Ts);
	}
	Ts=Ts*1000;

/* current limit */
	printf("\n I_limit (current limit [A]) (5.0A) :");
	scanf("%lf",&I_limit);

/* control variables */
	printf("\n Jn (nominal inertia [kgm2]) (0.04) :");
	scanf("%lf",&Jn);
	//printf("\n Tr (position response [ms]) (50ms) :");
	//scanf("%lf",&Tr);
	//Tr/=1000.0;
	//printf("\n Zt (damping factor) (1.0) :");
	//scanf("%lf",&Zt);
	printf("\n Kp :");
	scanf("%lf",&Kp);
	printf("\n Ki :");
	scanf("%lf",&Ki);
	printf("\n Kd :");
	scanf("%lf",&Kd);
	printf("\n Tau (derivative calq. [ms]) (5ms) :");
	scanf("%lf",&Tau);
	Tau /=1000.0;
	b0=Ki;
	b1=Kp+Ki*Tau;
	b2=Kp*Tau+Kd;
	c1=b0/Tau;
	c2=b1/Tau-b2/Tau/Tau;
	cu=b2/Tau;

/* printout of the control parameters */
	printf("\n motor constants :\n Jn = %f [kgm2], Ktn = %f [Nm/A]",Jn,Ktn);
	printf("\n position controller gain :\n Kp = %f [A/rad], Kd = %f [A/(rad/s)]",Kp,Kd);

/* set control loop counter and start control */
	printf("\n \n Operation Time [s] :");
	scanf("%d",&Tcon);
	Tcon = Tcon*1000000/Ts;
	tmpDataI=calloc(Tcon, sizeof(double));
	tmpDataX=calloc(Tcon, sizeof(double));
	tmpDataXref=calloc(Tcon, sizeof(double));

/* boad open ,initialize */	
	if(DaOpen(1)==-1||AdOpen(1)==-1){
	fprintf(stderr,"DA Open(1) failed\n");
	exit(1);
	}

	if(iopl(3)==-1){
	fprintf(stderr,"iopl failed\n");
	exit(1);
	}

	enc_init();
	cntnum=0;
	signal(SIGINT,ctlstop);
	Vout=0.0;

/* real time system open */
      
      if (art_enter(ART_PRIO_MAX, ART_TASK_PERIODIC, Ts) == -1) {
          perror("art_enter");
          exit(1);
      }
	T_smpl=(double)Ts/1000000;

/* DA open,initialize */
	dnum=1;

	res = DaGetSamplingConfig(dnum, &Conf);
	if(res){
		printf("DaGetSamplingConfig error: res=%x\n", res);
		DaClose(dnum);
		exit(EXIT_FAILURE);
	}

	Conf.ulChCount = 2;
	Conf.SmplChReq[0].ulChNo = 1;
	Conf.SmplChReq[0].ulRange = DA_5V;
	Conf.SmplChReq[1].ulChNo = 2;
	Conf.SmplChReq[1].ulRange = DA_5V;
	Conf.ulSamplingMode = DA_IO_SAMPLING;
	Conf.ulSmplRepeat = 1;
	Conf.fSmplFreq = 100000.0;
	res = DaSetSamplingConfig(dnum, &Conf);
	if(res){
		printf("DaSetSamplingConfig error: res=%x\n", res);
		DaClose(dnum);
		exit(EXIT_FAILURE);
	}
	
	res = DaClearSamplingData(dnum);
	if(res){
		printf("DaClearSamplingData error: ret=%x\n", res);
		DaClose(dnum);
		exit(EXIT_FAILURE);
	}

/* AD open, initialize */
	memset(&Smplreq, 0, sizeof(ADSMPLREQ));
	res = AdGetSamplingConfig(1,&Smplreq);
	if(res){
		printf("AdGetSamplingConfig error: res=%x\n", res);
		AdClose(dnum);DaClose(dnum);
		exit(EXIT_FAILURE);
	}
	Smplreq.ulChCount=1;
	Smplreq.SmplChReq[0].ulChNo=1;
	Smplreq.SmplChReq[0].ulRange=AD_10V;
	Smplreq.ulSingleDiff=AD_INPUT_SINGLE;
	Smplreq.ulSamplingMode = AD_IO_SAMPLING;
	Smplreq.ulSmplNum=1;
	ulpNum=1;
	Smplreq.fSmplFreq=10000.0;
	res = AdSetSamplingConfig(1,&Smplreq);
	if(res){
		printf("AdSetSamplingConfig error: res=%x\n", res);
		AdClose(dnum);DaClose(dnum);
		exit(EXIT_FAILURE);
	}
	
	res = AdClearSamplingData(dnum);
	if(res){
		printf("AdClearSamplingData error: res=%x\n", res);
		AdClose(dnum);DaClose(dnum);
		exit(EXIT_FAILURE);
	}
/* real time system start */

	for (i = 0; i < Tcon; ++i) {

	if(OutChar==1){
	OutChar=0;
	outb(OutChar, LP0_PORT);
	}
	else{
	OutChar=1;
	outb(OutChar, LP0_PORT);
	}

	theta1=read_theta(1);
	
	Data=Adtransfer(1);
	
	X_ref=Data*(1.0-exp(-T_smpl*(double)i));
	//	X_ref=Data;
	X=theta1;
	I_ref=control(X_ref,X);
	Vout=I_ref;
//	if(I_ref>=I_limit) Vout = I_limit;
//	if(I_ref<= -I_limit) Vout = -I_limit;
//20151006  add limitation on the current 
	if(I_ref>=I_limit) Vout = I_ref = I_limit;
	if(I_ref<= -I_limit) Vout = I_ref = -I_limit;
	Datransfer(1,Vout);

	tmpDataI[i]=I_ref;
	tmpDataXref[i]=X_ref;
	tmpDataX[i]=X;

         if (art_wait() == -1) {
		Datransfer(1,0.0);
		outb(0, LP0_PORT);
		free(tmpDataI);
		free(tmpDataX);
		free(tmpDataXref);

		perror("art_wait");
		exit(1);
          }
	Datransfer(2,X);
	}
	Datransfer(1,0.0);
	resfile=fopen("result_PID.csv","w+");
	for(i=0;i<Tcon;i++){
	fprintf(resfile,"%f %f %f %f\n",i*T_smpl,tmpDataI[i],tmpDataXref[i],tmpDataX[i]);
	}
	fclose(resfile);

        /////////////////////////////////////////////
        ///   BEEP SOUND ////////////////////////////
        /////////////////////////////////////////////

	for(i=0;i<600;i++){
	  if(art_wait()==-1){
		perror("art_wait");
		exit(1);
	  }
	  beep=inb(KBD_PORT);
	  beep=(i&1)? beep | SPK_BIT : beep & ~SPK_BIT;
	  outb(beep,KBD_PORT);
	}
	/////////////////////////////////////////////

	free(tmpDataI);
	free(tmpDataX);
	free(tmpDataXref);
/* real time system stop */
	ctlstop();


      return 0;
  }
