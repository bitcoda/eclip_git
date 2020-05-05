
#include "___headers_FB.h"

Uint16 Find_Sta_CH_Margine(Uint16 Ch_No, Uint16 ch_mar_per);
Uint16 Find_End_CH_Margine(Uint16 Ch_No, Uint16 ch_mar_per);
Uint16 Find_1mm_pxl(Uint16 Ch_No);
void Ch_Range_Set(void);
void Para_flash_init( void );
void Up_Down_Offset_Set(int offset);
void Start_PWM_cnt_Set(void);
//============================================================================================
//  flag initial - power reset
//--------------------------------------------------------------------------------------------
void reset_initial(void)
{

        jud.d.request       =0;                 //main request flag
        jud.d.request_start =0;                 //Process end
        jud.o.led_sig_fl =0;

        T.f.Sys_Req=0;
        T.f.illu_Single_check=0;

        //scan over_flow 확인
        scan_end_cnt=0;
        int_flag_cnt=0;

        //current check
        jud.f.Ir_off=0;
        jud.f.Ir_off_sta=0;
        jud.d.Ir_off_cnt = 0;
        jud.d.Ir_off_delay=0;

        // Scan Prepare
        jud.f.real_ch =0;
        Ir_Scan_ord = 0;
        Cycle_start_ch=0;
        new_Cycle_start_fl=0;

        //PARA_SET
        T.f.data_move=0;


}
//============================================================================================

//============================================================================================
//  Parameter initial - reflash
//--------------------------------------------------------------------------------------------
void initial_para(void)
{
	//system parameter reset process! - power reset / flash write

int i;
Uint16 *p_para;

//============================================================================================
//  illu_ord_blank[10]
//--------------------------------------------------------------------------------------------
//생략
//============================================================================================

//============================================================================================
// Flash Parameter Read
//--------------------------------------------------------------------------------------------
	p_para = &jud.p.base;
	parameter_read();

	//---- flash 초기값이면 default 값으로 flash write ----
	if((ReadBuffer[CURRENT_PAGE][0] == 0xffff) && (ReadBuffer[CURRENT_PAGE][1] == 0xffff)	){	//PG Download

		 Para_flash_init(  );
		 parameter_write(p_para, 0, FLASH_BUFF_SIZE);				//parameter all initial reset;  posi(0), start_no, length


	}
	//--------------------------------------------------------------------------------------------

	//============================================================================================
	//   parameter 변수에 값 할당
	//--------------------------------------------------------------------------------------------

	//변경 금지 파라미터 처리 CH_posi_fixed[PARA_CH_BUFF] -  1 ~ 10 ch. initial defined ch. posi value
	//필요없슴
    //--------------------------------------------------------------------------------------------

	for(i=0; i<FLASH_BUFF_SIZE;i++){                                // = PARA SIZE
		*p_para++ =  ReadBuffer[CURRENT_PAGE][i];
	}
	//--------------------------------------------------------------------------------------------
//============================================================================================


//============================================================================================
//  Judgment Parameter	#0 - #9
//--------------------------------------------------------------------------------------------
	//para #0 - #9
	if((jud.p.base 				< 40) 	||(jud.p.base 		> 100)) 	    jud.p.base			=	50;		//base level
	if( (jud.p.IR_volts 		> 330)  || (jud.p.IR_volts <130 ))		    jud.p.IR_volts		=	200;	//ir 조명 볼트( 0 ~ 200;2.0)

	if((jud.p.base_margine 		< 3) 	||(jud.p.base_margine > 50)) 	    jud.p.base_margine	=	10;

	if((jud.p.Start_PWM_cnt <	2590) 	||(jud.p.Start_PWM_cnt > 2610))     jud.p.Start_PWM_cnt=START_PWM;		//2600

	if((jud.p.Sync_Ch 		 >  END_CH_NO)) 	jud.p.Sync_Ch	=	0;											//

	if((jud.p.bg_cancel_multiple > 50))  jud.p.bg_cancel_multiple= 20;
	bg_multi = (float)jud.p.bg_cancel_multiple/10;

	//jud.p.sync_step에 Sync_Cycle도 함께 보냄
//	temp2 = jud.p.sync_step;

	if((jud.p.sync_step  >  END_CH_NO)  || (jud.p.sync_step < 1 ))           jud.p.sync_step   =   3;  //non_continous Sync에서 cycle 종료후 ch. step up - ver4.1
	                                                                                            //(CAM_Select_ord +1) * Cam_sync_step % 10
	if( (jud.p.Up_Down_Offset > 20)     || (jud.p.Up_Down_Offset< -20) || (jud.p.Up_Down_Offset == -1))     jud.p.Up_Down_Offset=0; //

	if((jud.p.Exep_Algo_Select<0)     ||  (jud.p.Exep_Algo_Select >1 )   )  jud.p.Exep_Algo_Select=0;         //0 - 시행 않함, 1 - 실시,peak exception Algolizm
	if( (jud.p.Exep_Area_Depth_Ratio_limit < 2) || (jud.p.Exep_Area_Depth_Ratio_limit > 50)) jud.p.Exep_Area_Depth_Ratio_limit = 20;    //peak exception에서 출력 peak로 판단하지 않는 기준, 이값 이상이 되면 피크를 제외한다

	//jud.p.reserv[0]=0;		//10개 채운다

	//Start_PWM_cnt_Set
	if(jud.p.Start_PWM_cnt != Start_PWM_cnt_old){
		Start_PWM_cnt_Set();
	}
	Start_PWM_cnt_old = jud.p.Start_PWM_cnt;
//============================================================================================
//  processing parameter #10 - #19 ; Ram Command 영역으로 사용
//--------------------------------------------------------------------------------------------
	//para #10 - #19
	if(jud.p.IR_order 			>END_CH_NO) 	 						jud.p.IR_order		=	0;	// ir 조명 순서  	0- 1,6 -> 2,7, -> 3,8 -> 4,9 -> 5,10
										// 기능 변경 - LED조명 검사용 at Sys_Control = 2
										// 0 - 모든 LED 순차점등 - 3초, 1 ~ 10 해당 번호 채널 계속 점등 - ram memory
	jud.p.Sys_Control =0;

	if((jud.p.CH_jud_level 		< 5) 	||(jud.p.CH_jud_level > 200)) 	jud.p.CH_jud_level	=	20;		//initial Ch. position 찾는 레벨 Margine


	//add 4.5
	if((jud.p.IR_order_term<20)     ||  (jud.p.IR_order_term >10000 )   )  jud.p.IR_order_term=500;         //0 - 시행 않함, 1 - 실시,peak exception Algolizm
	if((jud.p.IR_check_volts<50)     ||  (jud.p.IR_check_volts >300 )   )  jud.p.IR_check_volts=120;         //0 - 시행 않함, 1 - 실시,peak exception Algolizm
	//jud.p.IR_order_term;				//IR_order =0, ==> order step scan
	//jud.p.IR_check_volts;				//Sys_Control=2(LEd 조명 검사) - Ir volts

//	jud.p.IR_order_old=0;   4.5 삭제
//	jud.p.IR_order_test=0;  4.5 삭제


	jud.p.IR_volts_old=200;

	//초기 random 값 설정 -> jud.p.sync_step에 Sync_Cycle도 함께 보냄 v4,6
	//jud.p.Sync_Cycle_No = Random_Sync_Cycle_No();
	jud.p.Sync_Cycle_No = (temp2 & 0xff00)>>8;

	jud.p.reserv1[0] =0;       //10채운다
	jud.p.reserv1[1] =0;       //10채운다
	jud.p.reserv1[2] =0;
//============================================================================================
//  Ch. Position Default #20 - #29
//--------------------------------------------------------------------------------------------
#define CAM_MIN_PXL 	100
#define CAM_MAX_PXL  	1900

	fixed_Ch_Posi_set();		//fixed_posi() set 별도파일로 분리함

	//============================================================================================
	//  jud.p.CH_posi[i],  jud.p.CH_posi_init[i] default set ==> flash 값 제거가 목적
	//--------------------------------------------------------------------------------------------
	//CH_posi_fixed[i]  - 테스트후 동일하게 적용한 CH_posi
	//CH_posi_init[i]   - 생산시 설정된 CH_posi
	//CH_posi           - 실제 사용되는 CH_posi

	//jud.p.CH_posi_init[i]는 생산시 자동설정되게 한다. ==> jud.p.CH_posi[i]도 같이 설정
	//이 시점에서 jud.p.CH_posi_fixed[i], jud.p.CH_posi_init[i], jud.p.CH_posi[i] 가 설정되어 있슴
	//jud.p.CH_posi_init[i]설정은 생산에서 하며, 범위를 직접 확인하여야 하며, 범위내에서만 변경가능하다.
	//jud.p.CH_posi_init[i]은 현장에서 설정 가능하며, 범위내에서만 변경되게 한다.

	for(i=0; i<CAM_CH_NO ; i++){																//compile 후
		if( (jud.p.CH_posi[i] < CAM_MIN_PXL ) || (jud.p.CH_posi[i] > CAM_MAX_PXL) )
				jud.p.CH_posi[i] =jud.p.CH_posi_fixed[i];

		if( (jud.p.CH_posi_init[i] < CAM_MIN_PXL ) || (jud.p.CH_posi_init[i] > CAM_MAX_PXL) )
				jud.p.CH_posi_init[i] =jud.p.CH_posi_fixed[i];
	}
//============================================================================================


//============================================================================================
//  Ch_mar_percent	#30 - # 39 - 초기값을 결정해 준다.
//--------------------------------------------------------------------------------------------

	Ch_Margin_percent_set();											// 모듈 분리함
//============================================================================================

//============================================================================================
//   #40 - #49
//--------------------------------------------------------------------------------------------
//Uint16  CH_posi_init[PARA_CH_BUFF];	// 1 ~ 10 ch. position, manufacture initial set value
//============================================================================================

//============================================================================================
//   #50 - #59 CH_jud_level_add 		at v3_38
//--------------------------------------------------------------------------------------------
//int CH_jud_level_add[10];
//============================================================================================

//============================================================================================
//   #60 - #69 CH_IR_volts_add		at v3_38
//--------------------------------------------------------------------------------------------
//int CH_IR_volts_add[10];
//============================================================================================

//============================================================================================
//   #70 - #79
//--------------------------------------------------------------------------------------------
//  Ir_illumi_Single[PARA_CH_BUFF] ==> Send to illumi B/D

//	if( ILLU_SINGLE_CHECK
	//if(T.Rx_Data[3] == 70){
	if(T.Para_buff[3] == 70){       //single, double illumi
        for(i=0; i<CAM_CH_NO ; i++){
            if((jud.p.Ir_illumi_Single[i] < 0) || (jud.p.Ir_illumi_Single[i] > 1) )
            jud.p.Ir_illumi_Single[i] = 0;
        }

    	T.f.Sys_Req =1;
    	T.f.SpiAck	=1;
    	T.f.illu_Single_check =1;

	}
//============================================================================================

//============================================================================================
//   #80 - #89
//--------------------------------------------------------------------------------------------
//Uint16  CH_posi_fixed[PARA_CH_BUFF];	// 1 ~ 10 ch. initial defined ch. posi value
//============================================================================================

//============================================================================================
//   #90 - #99 - judgement Parameter
//--------------------------------------------------------------------------------------------
	if( (jud.p.jude_scan_cnt > 32) || (jud.p.jude_scan_cnt < 3) ) jud.p.jude_scan_cnt = 8;
	if( (jud.p.Out_on_cnt > jud.p.jude_scan_cnt) || (jud.p.Out_on_cnt < 1) )
		jud.p.Out_on_cnt = jud.p.jude_scan_cnt - 2;

	if( (jud.p.Out_off_cnt > (jud.p.jude_scan_cnt-2)) )
		jud.p.Out_off_cnt = jud.p.Out_on_cnt-2;

	if(jud.p.Blank_cycle_sw != 1) jud.p.Blank_cycle_sw = 0;	//default off
		//int    Reserv9[7];    // 1 ~ 10 ch. initial defined ch. posi value

		//ver4.91 -Acc judge
	if(jud.p.Acc_jud_on != 1) 		jud.p.Acc_jud_on = 0;	//Accuracy judge 1;on, default off
	if( (jud.p.Acc_jud_level > 50) || (jud.p.Acc_jud_level < 3) ) jud.p.Acc_jud_level = 7;
							//Accuracy 판단시 판단 레벨 1,on
	if( (jud.p.Acc_jud_ch == 0 ) || (jud.p.Acc_jud_ch > 0x3ff) ) jud.p.Acc_jud_ch = 0x303;	//11 0000 0011 1,2, 9,10 ch

	//v4.94 add - Diff_substance process
	if(  jud.p.Diff_sub_on != 0) 	jud.p.Diff_sub_on = 1;	//Diff_substance 1;on, default on
	if( (jud.p.Diff_sub_ch== 0 ) || (jud.p.Diff_sub_ch> 0x3ff) ) jud.p.Diff_sub_ch= 0x030;	//5, 6 ch
	if(jud.p.noise_shift != 0) jud.p.noise_shift = 1;
//	for(i=0; i<1; i++){
//		jud.p.Reserv9[i] =0;}       //10채운다
//============================================================================================

//============================================================================================
//  Ch. Range Set
//--------------------------------------------------------------------------------------------
//	Ch_Range_Set();											//ch_posi, Start, End, _1mm_pxl
//ver4,2
//	if(jud.p.Up_Down_Offset)
//	    Up_Down_Offset_Set(jud.p.Up_Down_Offset);          //jud.p.Up_Down_Offset; -2 ~ +2 , up;+, down; -, mm*10
//	else
	    Ch_Range_Set();                                         //ch_posi, Start, End, _1mm_pxl
//============================================================================================

//=========================================================
//	parameter re_assign - structure
//---------------------------------------------------------

		Vol_con_DAC101(jud.p.IR_volts);             //초기화 필요

		for(i=0; i<2080;i++){						//dlis_data2 initial
				dlis_data2[i] = 0;
			}

}
//============================================================================================

//============================================================================================
// Start_PWM_cnt_Set  - 노출시간 컨트롤
//--------------------------------------------------------------------------------------------
void Start_PWM_cnt_Set(void)
{

	//=========================================================
	//	PWM Sart Reset  - 노출시간 컨트롤
	//---------------------------------------------------------
		   DINT;
		   EPwm1Regs.TBPRD = 8*jud.p.Start_PWM_cnt;

			EINT;   				// Enable Global interrupt INTM
			ERTM;   				// Enable Global realtime interrupt DBGM
		   // Setup TBCLK
		  // EPWM1_TIMER_TBPRD;           // Set timer period 801 TBCLKs
	//============================================================================================

}
//============================================================================================

//============================================================================================
//  Ch. Range Set
//--------------------------------------------------------------------------------------------
void Ch_Range_Set(void)
{
//jud.d.Ch_range[10][4];		ch_posi, Start, end, _1mm_pxl
Uint16 i;

	for(i=0; i<CAM_CH_NO;i++){
		jud.d.Ch_range[i][0] = jud.p.CH_posi[i];
		jud.d.Ch_range[i][1] = jud.p.CH_posi[i] - Find_Sta_CH_Margine(i, jud.p.Ch_marg_percent[i] );		//Start,   - Start < End
		jud.d.Ch_range[i][2] = jud.p.CH_posi[i] + Find_End_CH_Margine(i, jud.p.Ch_marg_percent[i] );		//End
		jud.d.Ch_range[i][3] = Find_1mm_pxl(i);												//_1mm_pxl
	}
}
//============================================================================================

//============================================================================================
//  Find_Sta_CH_Margine
//--------------------------------------------------------------------------------------------
Uint16 Find_Sta_CH_Margine(Uint16 Ch_No, Uint16 ch_mar_per){
	//ch0_posi > ch1_posi인 것을 전제로 한다. 작은값 쪽의 ch_posi 의 1/2  ch_no > [ch_no-1] 를 조건으로 한다
	//Start - ch. Upper(채널으로 부터 위쪽 -큰 채널 쪽)

Uint16 Ch_Margine_=0;
long int Upper_mar=0;

	if( (ch_mar_per & 0xff00) != 0)
		 Upper_mar = (ch_mar_per>>8);
	else Upper_mar = ch_mar_per;


		if(Ch_No == END_CH_NO)
			Ch_Margine_ = (( (jud.p.CH_posi[END_CH_NO -1] - jud.p.CH_posi[END_CH_NO])/2)  * Upper_mar)/100;
		else
			Ch_Margine_ = (( (jud.p.CH_posi[Ch_No]   	  - jud.p.CH_posi[Ch_No+1])/2)    * Upper_mar)/100;

		return Ch_Margine_;

}
//============================================================================================

//============================================================================================
//  Find_End_CH_Margine
//--------------------------------------------------------------------------------------------
Uint16 Find_End_CH_Margine(Uint16 Ch_No, Uint16 ch_mar_per){//작은값 족의 ch_posi 의 1/2  ch_no > [ch_no-1] 를 조건으로 한다
	//End - ch. Lower(채널으로 부터 아래쪽 -작은 채널 쪽)
Uint16 Ch_Margine_=0;
long int Lower_mar=0;

	if( (ch_mar_per & 0x00ff) != 0)
		 Lower_mar = (ch_mar_per&0x00ff);
	else Lower_mar = ch_mar_per>>8;

		if(Ch_No == 0)
			Ch_Margine_ = (( (jud.p.CH_posi[0] - jud.p.CH_posi[1])/2) 		  * Lower_mar)/100;
		else
			//Ch_Margine_ = (( (jud.p.CH_posi[Ch_No-1]-jud.p.CH_posi[Ch_No])/2) * (long int)ch_mar_per)/100;
			Ch_Margine_ = (( (jud.p.CH_posi[Ch_No-1]-jud.p.CH_posi[Ch_No])/2) * Lower_mar)/100;
		return Ch_Margine_;
}
//============================================================================================

//============================================================================================
//  Find_1mm_pxl
//--------------------------------------------------------------------------------------------
Uint16 Find_1mm_pxl(Uint16 Ch_No){							//Start < End , ch_posi(0) > ch_posi(1)
	//jud.p.CH_posi_init 값을 기준으로 ch. 중간값과 중간값을 Pitch로 나눈다 => 1mm pxl 수를 구한다.
	//*10 int로 관리

int  _imm_pxl=0;
int  Range_start=0;
int	 Range_end  =0;

		if(Ch_No == 0){
			Range_start = ( jud.p.CH_posi_init[0] - (jud.p.CH_posi_init[0] - jud.p.CH_posi_init[1])/2);
			Range_end   = ( jud.p.CH_posi_init[0] + (jud.p.CH_posi_init[0] - jud.p.CH_posi_init[1])/2);
		}
		else if(Ch_No == END_CH_NO){
			Range_start = ( jud.p.CH_posi_init[END_CH_NO] - (jud.p.CH_posi_init[END_CH_NO -1] - jud.p.CH_posi_init[END_CH_NO])/2);
			Range_end   = ( jud.p.CH_posi_init[END_CH_NO] + (jud.p.CH_posi_init[END_CH_NO -1] - jud.p.CH_posi_init[END_CH_NO])/2);
		}
		else 			{
			Range_start = ( jud.p.CH_posi_init[Ch_No] - (jud.p.CH_posi_init[Ch_No] 	- jud.p.CH_posi_init[Ch_No+1])/2);
			Range_end   = ( jud.p.CH_posi_init[Ch_No] + (jud.p.CH_posi_init[Ch_No-1] 	- jud.p.CH_posi_init[Ch_No])/2);
		}

		_imm_pxl = ((Range_end - Range_start)*10)/CH_PITCH;						//*10 Uint
		return _imm_pxl;
}
//============================================================================================

void Up_Down_Offset_Set(int offset)     //mm*10
{

   // jud.p.Up_Down_Offset; -2 ~ +2 , up;+, down; -, mm*10
    //jud.d.Ch_range[10][4];        ch_posi, Start, end, _1mm_pxl

    int i;

        //1mm_pxl set
        for(i=0; i<CAM_CH_NO;i++){
            jud.d.Ch_range[i][3] = Find_1mm_pxl(i);  //*10
        }
        //CH_posi set
        for(i=0; i<CAM_CH_NO;i++){
 //           jud.p.CH_posi[i] = jud.p.CH_posi_init[i] + (jud.p.Up_Down_Offset * jud.d.Ch_range[i][3])/(10*10);   //*10, *10
            jud.p.CH_posi[i] = jud.p.CH_posi[i] + (jud.p.Up_Down_Offset * jud.d.Ch_range[i][3])/(10*10);   //*10, *10

        }

        for(i=0; i<CAM_CH_NO;i++){

            jud.d.Ch_range[i][1] = jud.p.CH_posi[i] - Find_Sta_CH_Margine(i, jud.p.Ch_marg_percent[i] );    //Start,   - Start < End
            jud.d.Ch_range[i][2] = jud.p.CH_posi[i] + Find_End_CH_Margine(i, jud.p.Ch_marg_percent[i] );        //End

        }

}

//============================================================================================
//  Para_Flash_init   - flash download 후 initial parameter set  (0xffff)
//--------------------------------------------------------------------------------------------
void Para_flash_init( void ){													//compile 후
//flash 초기값이면 default 값으로 flash write
	//============================================================================================
	//  Judgment Parameter	#0 - #9
	//--------------------------------------------------------------------------------------------
		jud.p.base			=	50;			//base level
		jud.p.IR_volts		=	200;		//ir 조명 볼트( 0 ~ 200;2.0)
		jud.p.base_margine	=	10;
		jud.p.Start_PWM_cnt =	2600;		//채널마진 % ; 5 - 120
		jud.p.Sync_Ch		=	0;
		jud.p.bg_cancel_multiple= 20;
		jud.p.sync_step=2;
		jud.p.Up_Down_Offset=0;
		//Uint16  reserv[4];		//10개 채운다.

	//============================================================================================
	//  processing parameter #10 - #19
	//--------------------------------------------------------------------------------------------
		jud.p.IR_order		=	0;
		jud.p.Sys_Control	= 	0;
		jud.p.CH_jud_level	=	20;				//initial find ch. peak  level
		jud.p.Sys_Control 	=	0;

	    int Exep_Algo_Select=1;               //peak exception Algolizm 0 - 시행 않함, 1 - 실시
	    int Exep_Area_Depth_Ratio_limit=20;    //peak exception에서 출력 peak로 판단하지 않는 기준, 이값 이상이 되면 피크를 제외한다

	//============================================================================================
	//  CH_posi Default #20 - #29
	//--------------------------------------------------------------------------------------------
		fixed_Ch_Posi_set();				//fixed_posi() set 별도파일로 분리함

		for(i=0; i<CAM_CH_NO ; i++){
			jud.p.CH_posi[i] 		=jud.p.CH_posi_fixed[i];
		}

	//============================================================================================

	//============================================================================================
	//  Ch_mar_percent	#30 - # 39 - 초기값을 결정해 준다.
	//--------------------------------------------------------------------------------------------
		Ch_Margin_percent_set();											// 모듈 분리함
	//============================================================================================

	//============================================================================================
	//   #40 - #49
	//--------------------------------------------------------------------------------------------
	//Uint16  CH_posi_init[PARA_CH_BUFF];	// 1 ~ 10 ch. position, manufacture initial set value
	for(i=0; i<CAM_CH_NO ; i++){
		jud.p.CH_posi_init[i] 	=jud.p.CH_posi_fixed[i];
	}
	//============================================================================================


	//============================================================================================
	//   #50 - #59 CH_jud_level_add 		at v3_38
	//--------------------------------------------------------------------------------------------
	//int CH_jud_level_add[10];
	for(i=0; i<CAM_CH_NO ; i++){
		jud.p.CH_jud_level_add[i] 	=0;
	}
	//============================================================================================

	//============================================================================================
	//   #60 - #69 CH_IR_volts_add		at v3_38
	//--------------------------------------------------------------------------------------------
	//int CH_IR_volts_add[10];
	for(i=0; i<CAM_CH_NO ; i++){
		jud.p.CH_IR_volts_add[i] 	=0;
	}
	//============================================================================================

	//============================================================================================
	//   #70 - #79
	//--------------------------------------------------------------------------------------------
	//Uint16  CH_posi_fixed[PARA_CH_BUFF];	// 1 ~ 10 ch. initial defined ch. posi value
		fixed_Ch_Posi_set();
	//============================================================================================

}



//============================================================================================
// Random_Sync_Cycle_No - v4.6 Control에서 동일하게 결정
//--------------------------------------------------------------------------------------------
/*
int Random_Sync_Cycle_No(void)
{
    //random ran_sync_Cycle_no - 20 ~ 50

int ran_sync_Cycle_no=30;

    ran_sync_Cycle_no =  EPwm1Regs.TBCTR%30;

    ran_sync_Cycle_no = ran_sync_Cycle_no + 20;;


    return ran_sync_Cycle_no;

 }  //END
 */
//============================================================================================



