

#include "svpwm.h"
#include "math.h"

#define PI 3.1416

BLDC_SVPWMTypeDef svpwm1;


void Clark_Transformation(float* In_a, 	float* In_b, float* In_c, float* Out_alpha, float* Out_beta)
{
	(*Out_alpha) = (*In_a)*0.6667  - (*In_b)*0.3333 - (*In_c)*0.3333;
	(*Out_beta) = ((*In_b) - (*In_c)) * 0.5774;
}
void InvClark_Transformation(float* In_alpha, 	float* In_beta, float* Out_a, float* Out_b, float* Out_c)
{
	(*Out_a) = (*In_alpha);
	(*Out_b) = -0.5 * (*In_alpha) + 0.8660 * (*In_beta);
	(*Out_c) = -0.5 * (*In_alpha) - 0.8660 * (*In_beta);	
}
void sectorJudge(BLDC_SVPWMTypeDef* svpwm)
{
	svpwm->b_VrefAngle = atan2(svpwm->b_VrefBeta, svpwm->b_VrefAlpha);
	
	if(( svpwm->b_VrefAngle >= 0.0) && ( svpwm->b_VrefAngle < PI/3.0))
	{
		svpwm->b_sector = 1;
	}
	else if(( svpwm->b_VrefAngle >= PI/3.0) && ( svpwm->b_VrefAngle < 2.0*PI/3.0))
	{
		svpwm->b_sector = 2;
	}
	else if(( svpwm->b_VrefAngle >= 2.0*PI/3.0) && ( svpwm->b_VrefAngle < PI))
	{
		svpwm->b_sector = 3;
	}
	else if(( svpwm->b_VrefAngle >= -PI/3.0) && ( svpwm->b_VrefAngle < 0.0))
	{
		svpwm->b_sector = 6;
	}
	else if(( svpwm->b_VrefAngle >= -2.0*PI/3.0) && ( svpwm->b_VrefAngle < -PI/3.0))
	{
		svpwm->b_sector = 5;
	}
	else if(( svpwm->b_VrefAngle >= -PI) && ( svpwm->b_VrefAngle < -2.0*PI/3.0))
	{
		svpwm->b_sector = 4;
	}
	else
	{
		while(1); //computation error
	}
}
int32_t sectorJudge_v2(BLDC_SVPWMTypeDef* svpwm)
{
	float U1,U2,U3;
	int32_t A,B,C,N,b_sector1;
	
	U1 = svpwm->b_VrefBeta;
	U2 = 0.8660*svpwm->b_VrefAlpha - svpwm->b_VrefBeta*0.5;
	U3 = -0.8660*svpwm->b_VrefAlpha - svpwm->b_VrefBeta*0.5;

	if(U1>0)	{
		A=1;
	}
	else	{
		A=0;
	}
	if(U2>0)	{
		B=1;
	}
	else	{
    		B=0;
	}
	if(U3>0)	{
		C=1;
	}
	else	{
    		C=0;
	}
    
	N = 4*C +2*B +A;
	if( N==3)	{
		b_sector1 = 1;
	}
	else if( N==1){		
    		b_sector1 = 2;
	}
	else if( N==5)	{
    		b_sector1 = 3;
	}
	else if( N==4)	{
    		b_sector1 = 4;
	}
	else if (N==6)	{
    		b_sector1 = 5;
	}
	else if( N==2)	{
    		b_sector1 = 6;
	}	
	return b_sector1;
}
void SpaceVectorUpdate(BLDC_SVPWMTypeDef* svpwm)
{
	switch(svpwm->b_sector)
	{
		case 1:
			svpwm->a_VectorOut[0] = 4;
			svpwm->a_VectorOut[1] = 6;
			svpwm->a_VectorOut[2] = 0;
			
			svpwm->a_VectorOutDuty[0] = svpwm->b_VrefAlpha - svpwm->b_VrefBeta*0.57735;
			svpwm->a_VectorOutDuty[1] = svpwm->b_VrefBeta*1.1547;
			svpwm->a_VectorOutDuty[2] = 0;		
			break;
		case 2:
			svpwm->a_VectorOut[0] = 2;
			svpwm->a_VectorOut[1] = 6;
			svpwm->a_VectorOut[2] = 0;
			
			svpwm->a_VectorOutDuty[0] = -svpwm->b_VrefAlpha + svpwm->b_VrefBeta*0.57735;
			svpwm->a_VectorOutDuty[1] =  svpwm->b_VrefAlpha + svpwm->b_VrefBeta*0.57735;
			svpwm->a_VectorOutDuty[2] = 0;		
			break;
		case 3:				
			svpwm->a_VectorOut[0] = 2;
			svpwm->a_VectorOut[1] = 3;
			svpwm->a_VectorOut[2] = 0;
			
			svpwm->a_VectorOutDuty[0] = svpwm->b_VrefBeta*1.1547;
			svpwm->a_VectorOutDuty[1] = -svpwm->b_VrefAlpha - svpwm->b_VrefBeta*0.57735;
			svpwm->a_VectorOutDuty[2] = 0;		
			break;
		case 4:				
			svpwm->a_VectorOut[0] = 1;
			svpwm->a_VectorOut[1] = 3;
			svpwm->a_VectorOut[2] = 0;
			
			svpwm->a_VectorOutDuty[0] = -svpwm->b_VrefBeta*1.1547;
			svpwm->a_VectorOutDuty[1] = -svpwm->b_VrefAlpha + svpwm->b_VrefBeta*0.57735;
			svpwm->a_VectorOutDuty[2] = 0;		
			break;
		case 5:
				
			svpwm->a_VectorOut[0] = 1;
			svpwm->a_VectorOut[1] = 5;
			svpwm->a_VectorOut[2] = 0;
			
			svpwm->a_VectorOutDuty[0] = -svpwm->b_VrefAlpha - svpwm->b_VrefBeta*0.57735;
			svpwm->a_VectorOutDuty[1] =  svpwm->b_VrefAlpha - svpwm->b_VrefBeta*0.57735;
			svpwm->a_VectorOutDuty[2] = 0;		
			break;
		case 6:
				
			svpwm->a_VectorOut[0] = 4;
			svpwm->a_VectorOut[1] = 5;
			svpwm->a_VectorOut[2] = 0;
			
			svpwm->a_VectorOutDuty[0] = svpwm->b_VrefAlpha + svpwm->b_VrefBeta*0.57735;
			svpwm->a_VectorOutDuty[1] = -svpwm->b_VrefBeta*1.1547;
			svpwm->a_VectorOutDuty[2] = 1.0- svpwm->a_VectorOutDuty[0] - svpwm->a_VectorOutDuty[1];		
			break;
		default:
			break;
	}
	
	svpwm->a_VectorOutDutyCmpr[0] = svpwm->a_VectorOutDuty[2] * 14399/2.0;
	svpwm->a_VectorOutDutyCmpr[1] = svpwm->a_VectorOutDuty[0] * 14399/2.0;
	svpwm->a_VectorOutDutyCmpr[2] = svpwm->a_VectorOutDuty[1] * 14399;
	svpwm->a_VectorOutDutyCmpr[3] = svpwm->a_VectorOutDuty[0] * 14399/2.0;
	
	svpwm->a_VectorOutDutyCmpr[1] += svpwm->a_VectorOutDutyCmpr[0];
	svpwm->a_VectorOutDutyCmpr[2] += svpwm->a_VectorOutDutyCmpr[1];
	svpwm->a_VectorOutDutyCmpr[3] += svpwm->a_VectorOutDutyCmpr[2];
	
	svpwm->OutA = 			svpwm->a_VectorOutDutyCmpr[1]/15000;
	svpwm->OutB = 			svpwm->a_VectorOutDutyCmpr[2]/15000;
	svpwm->OutC = 			svpwm->a_VectorOutDutyCmpr[3]/15000;
}
void Periodic()
{

		svpwm1.b_VrefA  = 0.8660 * 1000 * cos(6.2832*svpwm1.b_TIM1PrdCnt/1000.0);
		svpwm1.b_VrefB  = 0.8660 * 1000 * cos(6.2832*svpwm1.b_TIM1PrdCnt/1000.0 - 6.2832/3.0);
		svpwm1.b_VrefC  = 0.8660 * 1000 * cos(6.2832*svpwm1.b_TIM1PrdCnt/1000.0 + 6.2832/3.0);
		
		// 0 <= Vm <=1
		// to avoid over modulation, scale down the Vref
		//svpwm1.b_VrefA *= 0.8660;
		//svpwm1.b_VrefB *= 0.8660;
		//svpwm1.b_VrefC *= 0.8660;
		
		Clark_Transformation(&svpwm1.b_VrefA, &svpwm1.b_VrefB, &svpwm1.b_VrefC, &svpwm1.b_VrefAlpha, &svpwm1.b_VrefBeta);
		sectorJudge(&svpwm1);
		SpaceVectorUpdate(&svpwm1);
		
		//TIM1CmprLoad();
		
		//SVPWM1_SpaceVectorDRV(svpwm1.a_VectorOut[2]);
		
		svpwm1.b_TIM1PrdCnt++;
		if(svpwm1.b_TIM1PrdCnt==1000)
		{
			svpwm1.b_TIM1PrdCnt = 0;
		}

}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		//SVPWM1_SpaceVectorDRV(svpwm1.a_VectorOut[0]);		
	}
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		//SVPWM1_SpaceVectorDRV(svpwm1.a_VectorOut[1]);				
	}
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		//SVPWM1_SpaceVectorDRV(svpwm1.a_VectorOut[0]);			
	}
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
		//SVPWM1_SpaceVectorDRV(svpwm1.a_VectorOut[2]);			
	}

}
