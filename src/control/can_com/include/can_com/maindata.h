//#ifndef _main_H_
//#define _main_H_



// extern void QLViewVehicle_Data_Input();
// extern void QLViewVehicle_CANDATA_Ma();
// extern void QLViewVeh_Control_Output();
extern void JMEV_Data_Input();
extern void JMEV_SIGNAL_Parsed();
extern void JMEV_Control_Output();
extern void Vehicle_Platform();
extern void Remote_Control();
extern void VehiclePosInit();

//extern uint8_t JMEVmsg_ViewcarCtrl[78];
//extern uint8_t JMEVmsg_ViewcarCtrl[39];
extern struct can_frame  JMEVmsg_ViewcarCtrl[8];

extern struct can_frame  ViewVehicleData;
extern uint8_t VehCANDATA[650];
extern uint8_t msg_ViewcarCtrl[52];
extern uint8_t msg_Platfom[13];
extern uint8_t msg_FromPlatfom[650];
extern uint8_t msg[52];
extern uint16_t control_count;
extern uint16_t fusion_count;
 

