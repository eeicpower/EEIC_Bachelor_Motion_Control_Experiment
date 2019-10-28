# EEIC_Bachelor_Motion_Control_Experiment
EEICの後期実験で利用する制御プログラムを公開しています。  
最新リリース版を利用してください。
## プログラム構成
![DirTree](https://github.com/eeicpower/EEIC_Bachelor_Motion_Control_Experiment/blob/pict/pict/programConf.PNG)
                                                    
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
  ## AnalysisCode
  解析などに利用可能なコードを公開しています。
  ### sys_ident.m
  Matlabで動作可能なチャープ信号入力によるシステム同定プログラム
  ### ControllerDesign
  モータに関するパラメータからPD,PID制御器の制御ゲインを決めるための制御器設計プログラム
