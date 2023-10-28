
#include "can.h"
#include "bsp_can.h"


moto_measure_t moto_chassis[4] = {0};//4 chassis moto
static CAN_TxHeaderTypeDef	TXHeader;
static CAN_RxHeaderTypeDef 	RXHeader;
uint8_t TXmessage[8];
uint8_t RXmessage[8];
uint32_t pTxMailbox = 0;
int flag=0;



void get_total_angle(moto_measure_t *p);
void get_moto_offset(moto_measure_t *ptr, uint8_t *RxMessage);

/*******************************************************************************************
  * @Func		my_can_filter_init
  * @Brief    CAN1?CAN2?????
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2015/11/30
 *******************************************************************************************/
void my_can_filter_init_recv_all(CAN_HandleTypeDef* hcan)
{

    CAN_FilterTypeDef  can_filter;

    can_filter.FilterBank = 0;                       // filter 0
    can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // ????????
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;		//????????32?
    can_filter.FilterIdHigh = 0;//?????? 
    can_filter.FilterIdLow  = 0;//?????? 
    can_filter.FilterMaskIdHigh = 0;//?????
    can_filter.FilterMaskIdLow  = 0;       //?????   set mask 0 to receive all can id
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0,????FIFO0
    can_filter.FilterActivation = ENABLE;           // enable can filter
    can_filter.SlaveStartFilterBank  = 14;          
   
    HAL_CAN_ConfigFilter(&hcan1, &can_filter);        // init can filter
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);		//??can?FIFO0??
    HAL_CAN_Start(&hcan1);//??can1

}

uint32_t FlashTimer;
/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    HAL?????CAN????????,????????CAN????????
  * @Param		
  * @Retval		None 
  * @Date     2015/11/24
 *******************************************************************************************/
void HAL_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  flag=1;
	if(HAL_GetTick() - FlashTimer>500)
	{
		FlashTimer = HAL_GetTick();	
	}
	HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RXHeader,RXmessage);
	//ignore can1 or can2.
	static u8 i;
	i = RXHeader.StdId - CAN_2006Moto1_ID;
	
	get_moto_measure(&moto_chassis[i], RXmessage);
	// __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_RX_FIFO0);
}


/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    ??3508????CAN??????
  * @Param		 
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr, uint8_t *RxMessage)
{
	
	//获取数据
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(RxMessage[0]<<8 | RxMessage[1]) ;
	ptr->speed_rpm  = (int16_t)(RxMessage[2]<<8 | RxMessage[3]);
	ptr->real_current = (RxMessage[4]<<8 | RxMessage[5])*5.f/16384.f;

	ptr->hall = RxMessage[6];
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, uint8_t *RxMessage)
{
	ptr->angle = (uint16_t)(RxMessage[0]<<8 | RxMessage[1]) ;
	ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
*@bref ??????=0, ?????????3510????????(?0)??????
	*/
void get_total_angle(moto_measure_t *p){
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//?????
		res1 = p->angle + 8192 - p->last_angle;	//??,delta=+
		res2 = p->angle - p->last_angle;				//??	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//??	delta -
		res2 = p->angle - p->last_angle;				//??	delta +
	}
	//?????,??????????????
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}

void set_moto_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4){

	TXHeader.StdId = 0x200;
	TXHeader.IDE = CAN_ID_STD;
	TXHeader.RTR = CAN_RTR_DATA;
	TXHeader.DLC = 0x08;
	TXmessage[0] = (iq1 >> 8);
	TXmessage[1] = iq1;
	TXmessage[2] = (iq2 >> 8);
	TXmessage[3] = iq2;
	TXmessage[4] = iq3 >> 8;
	TXmessage[5] = iq3;
	TXmessage[6] = iq4 >> 8;
	TXmessage[7] = iq4;
	
	int FreeTxNum =0;
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
	while(FreeTxNum == 0)FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
	/*等待发送邮箱有空*/
	
	HAL_CAN_AddTxMessage(&hcan1, &TXHeader,TXmessage,(uint32_t*)CAN_TX_MAILBOX0);
}	

void set_Cascademoto_current(CAN_HandleTypeDef* hcan, s16 iq){

	TXHeader.StdId = 0x200;
	TXHeader.IDE = CAN_ID_STD;
	TXHeader.RTR = CAN_RTR_DATA;
	TXHeader.DLC = 0x08;
	TXmessage[0] = (iq >> 8);
	TXmessage[1] = iq;
	
	int FreeTxNum =0;
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
	while(FreeTxNum == 0)FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
	/*等待发送邮箱有空*/
	
	HAL_CAN_AddTxMessage(&hcan1, &TXHeader,TXmessage,(uint32_t*)CAN_TX_MAILBOX0);
}	
