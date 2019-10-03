# EEIC_Bachelor_Motion_Control_Experiment
EEICの後期実験で利用する制御プログラムを公開しています。
## プログラム構成
src---BasicControlPrograms----Makefile  
    |                      |--inert_external.c  
    |                      |--inert_internal.c  
    |                      |--pd4.c  
    |                      |--dist4.c  
    |  
    |-AdvancedControlPrograms---ChirpIdentification----Makefile  
                              |                     |--inert_chirp.c  
                              |                     |--chirp.c  
                              |                     |--chirp.h  
                              |  
                              |--SimplePID-------------Makefile  
                                                    |--pid.c  
                                                    
## BasicControlPrograms
　実験のうち必ず行う制御に関するプログラムが入っています。
### Makefile
　プログラムをコンパイルするためのファイルです，
  'make'で以下のプログラムをすべてコンパイルします。
### inert_X.c
　システム同定の実験で利用します。
* X=external: 外部のファンクションジェネレータからトルク指令を受け取ります
* X=internal: プログラム内部で正弦波トルク指令を生成します
### pd4.c
　PD制御を行うときに利用します。  
 ファンクションジェネレータから位置指令を受け取ります。
### dist4.c
　外乱オブザーバの実験で利用します。  
 ファンクションジェネレータから位置指令を受け取ります。
 
## AdvancedControlPrograms
　実験のうち発展/考察課題で利用します。
### ChirpIdentification
　システム同定をチャープ信号によって行うときに利用します。  
 chirp.cを修正することで様々なチャープ信号を利用できます。  
 プログラム内部で正弦波トルク指令を生成します
 ### SimplePID
 　PID制御を行うときに利用します。  
  ファンクションジェネレータから位置指令を受け取ります。
