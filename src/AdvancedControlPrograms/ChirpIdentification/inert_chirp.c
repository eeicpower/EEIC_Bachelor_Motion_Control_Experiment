//inert.c
#include <stdlib.h>
#include <stdio.h>
#include <sys/io.h>
#include <sys/signal.h>
#include <math.h>
#include <fbida.h>
#include <fbiad.h>
#include "/usr/src/linux-2.4.20-ART/include/linux/art_types.h" /*  iji */
#include "/usr/src/linux-2.4.20-ART/include/linux/art_task.h" /* iji  */
#include "chirp.h"

#define TRUE 1

#define KBD_PORT 0x61
#define SPK_BIT  0x02
#define LP0_PORT 0x378

#define ADADRES0 0x9000
#define DAADRES0 0x9400
#define ENCADRES0 0x9800

#define ENC_RESL 312500.0

int Tcon, Ts;
double V_limit;

/* motor constants */
double Jn; //[kgm^2]
double Ktn = 1.8; //[Nm/V]
	//motor is converted in 2015
	// D/A output:-5[V]~5[V]
	// Corresponding torque:-9[Nm]~9[Nm]
double Max_V = 5.0;
	// Maximum voltage reference of motor driver

/* controller variables definition */
/* controller variables definition */
double X;
double Freq, ang_freq, Va; // Freq: reference current frequency, ang_freq: angular frequency 
double X, X_ref;
double T_smpl=0.0, Time = 0.0;
double fmin, fmax, charptime;


///////////* encorder initialize *////////////
int enc_init(){
	fprintf(stderr,"Encoder ADRESS: %4x\n",ENCADRES0);
	outb(0x6,ENCADRES0 + 0x04);
	outb(0x6,ENCADRES0 + 0x14);
	return 0;
}
//////////////////////////////////////////////

///////////* Ana->Dig transfer *//////////////
/*
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
*/
//////////////////////////////////////////////

///////////* Dig->Ana transfer *//////////////
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
//////////////////////////////////////////////


/////////////* read encorder *////////////////
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
	//motor is converted to new version in 2015
	return(rad);
}
//////////////////////////////////////////////

/////////////* real time stop *///////////////
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
//////////////////////////////////////////////


//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//						MAIN PROGRAM 						// 
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
	int i, cntnum,beep;
	unsigned char 	OutChar=0;
	double X, Vout, w=0.0, W=0.0, X_old=0.0;
	int res,dnum;
	unsigned long ulpNum;
	ADSMPLREQ Smplreq;
	DASMPLREQ Conf;
	FILE *resfile;
	double *tmpDataV, *tmpDataW;

	///////////////////////////////////////////////
	////////////////* sampling time *//////////////
	///////////////////////////////////////////////
	Ts=0;
	while(Ts <= 0){
		printf("\n sampling time [us] (1000 us):");
		scanf("%d",&Ts);
	}
	
	///////////////////////////////////////////////
	////////////////* current limit *//////////////
	///////////////////////////////////////////////
	V_limit=-1;
	while(V_limit < 0 || V_limit > Max_V){
		printf("\n Torque limit [Nm] (9.0 Nm) :");
		scanf("%lf",&V_limit);
	}
	V_limit/=Ktn; 


	///////////////////////////////////////////////
	////////////* current amplitude *//////////////
	///////////////////////////////////////////////
	Va=-1;
	while(Va < 0 || Va > Max_V){
		printf("\n Chirp amplitude [V] (2 V) :");
		scanf("%lf",&Va);
	}
	

	///////////////////////////////////////////////
	////////////* chirp parameters *///////////////
	///////////////////////////////////////////////
	printf("\n min Freq (frequency [Hz]) (0.1[Hz]) :");
	scanf("%lf",&fmin);
	printf("\n max Freq (frequency [Hz]) (50[Hz]) :");
	scanf("%lf",&fmax);
	printf("\n chirp time (time [s]) (5[s]) :");
	scanf("%lf",&charptime);
	
	///////////////////////////////////////////////
	/////* printout of the control parameters *////
	///////////////////////////////////////////////
	printf("\n motor constants :\n Ktn = %f [Nm/V]",Ktn);
	
	///////////////////////////////////////////////
	//set control loop counter and start control///
	///////////////////////////////////////////////
	Tcon = -1;
	while(Tcon < 0){
		printf("\n \n Operation Time [s] :");
		scanf("%d",&Tcon);
	}
	Tcon = Tcon*1000000/Ts;

	tmpDataV=calloc(Tcon, sizeof(double));
	tmpDataW=calloc(Tcon, sizeof(double));

	////////////////////////////////////////////
	//////* boad open ,initialize */////////////
	////////////////////////////////////////////
	if(DaOpen(1)==-1||AdOpen(1)==-1){
		fprintf(stderr,"DA Open(1) failed\n");
		exit(1);
	}

	// I/O permission change 
	if(iopl(3)==-1){
		fprintf(stderr,"iopl failed\n");
		exit(1);
	}

	enc_init();
	cntnum=0;
	signal(SIGINT,ctlstop);

	////////////////////////////////////////////
	//////* real time system open */////////////
	////////////////////////////////////////////
      
	if (art_enter(ART_PRIO_MAX, ART_TASK_PERIODIC, Ts) == -1) {
		perror("art_enter");
		exit(1);
	}
	T_smpl=(double)Ts/1000000;

	////////////////////////////////////////////
	//////* DA open, initialize *///////////////
	////////////////////////////////////////////

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

	////////////////////////////////////////////
	//////* AD open, initialize *///////////////
	////////////////////////////////////////////

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
	////////////////////////////////////////////
	//////* real time system start *////////////
	////////////////////////////////////////////

	for (i = 0; i < Tcon; ++i) {

		if(OutChar==1){
		OutChar=0;
		outb(OutChar, LP0_PORT);
		}
		else{
		OutChar=1;
		outb(OutChar, LP0_PORT);
		}

		X=read_theta(1);
		
		Time += T_smpl;
		ctrl_ident_chirplin(Time,&Vout,fmin,fmax,charptime);// make chirp signal

		Vout*=Va;

		if(Vout>=V_limit) Vout = V_limit;
		if(Vout<=-V_limit) Vout = -V_limit;

		Datransfer(1,Vout);//Output signal

		W=(X-X_old)/T_smpl;//angular speed
		X_old=X;

		tmpDataW[i]=W;
		tmpDataV[i]=Vout;
		Datransfer(2,W/10.0);
		
		if (art_wait() == -1) {
			Datransfer(1,0.0);
			outb(0, LP0_PORT);
			free(tmpDataV);
			free(tmpDataW);

			perror("art_wait");
			exit(1);
		}
	}


	Datransfer(1,0.0);
	resfile=fopen("result_chirp.csv","w+");
	printf("\n File format: Time, Voltage(Torque reference), Angular speed\n");
	printf("File exporting... \n");
	for(i=0;i<Tcon;i++){
		fprintf(resfile,"%f %f %f\n",i*T_smpl,tmpDataV[i],tmpDataW[i]);
		//File format: Time, Voltage(Torque), Angular speed
	}
	fclose(resfile);

	free(tmpDataV);
	free(tmpDataW);

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

	/////////////////////////////////////////////
	///////* real time system stop */////////////
	/////////////////////////////////////////////
	ctlstop();

	return 0;
  }







































































































































