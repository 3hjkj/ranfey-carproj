/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#include "ssFrameLib.h"
#include "string.h"
#include <rclcpp/rclcpp.hpp>
#include "ioapi.h"

int swapchar( unsigned char * _data, int size_ ) {
  int i = 0 , j = 0;
  char tmp = 0 ;
  for ( i=0, j= size_ - 1; i < size_ / 2 ; i++ , j--){
    tmp = _data[i] ;
    _data[i] = _data[j];
    _data[j] = tmp ;
  }
  return 0;
}

int checkSum(unsigned char * _dataBuf, int count_ ) {
  int rtn = 0 ;
  for( int i = 0 ; i < count_ ; i++ ) {
    rtn += _dataBuf[ i ] ;
  }
  rtn = rtn & 0xFF ;
  return rtn ;
}


void bufferToStruck(lidarAPi::FRAMS_BUFFER_S *mtFrameMsgBuf, void * mtframe, int mtFrameSize) {
  unsigned char* _tmp_buffer = NULL;
  _tmp_buffer = (unsigned char*)mtframe;
  int tmpIdx = mtFrameMsgBuf->readIdx;
  for (int i = 0; i < mtFrameSize; i++) {
    _tmp_buffer[i] = mtFrameMsgBuf->msgStream[tmpIdx];
    tmpIdx = (tmpIdx + 1) % FRAME_MSG_LENGTH;
  }
  return;
}

void writeFrameBuffer(lidarAPi::FRAMS_BUFFER_S *mtFrameMsgBuf, char * _mt_frame, int mt_size) {
  for (int i = 0; i < mt_size; i++) {
    mtFrameMsgBuf->msgStream[mtFrameMsgBuf->writeIdx] = _mt_frame[i];
    mtFrameMsgBuf->writeIdx = (mtFrameMsgBuf->writeIdx + 1) % FRAME_MSG_LENGTH;
    mtFrameMsgBuf->length++;
  }
  return;
}

void readDEBFrameBuffer(lidarAPi::FRAMS_BUFFER_S *mtFrameMsgBuf, lidarAPi::DEB_FRAME_S *mtRegMap) {
  lidarAPi::DEB_FRAME_S tmp_frame;
  int tmp_proLength = sizeof(lidarAPi::DEB_FRAME_S);
  while (mtFrameMsgBuf->length >= tmp_proLength) {
    bufferToStruck(mtFrameMsgBuf, (void*)&tmp_frame, tmp_proLength);
    switch (tmp_frame.msgHead) {
    case DEB_FRAME_READ:
      mtRegMap[tmp_frame.regAddress%ROMREG_MAX_COUNT] = tmp_frame;
      mtFrameMsgBuf->readIdx = (mtFrameMsgBuf->readIdx + tmp_proLength) % FRAME_MSG_LENGTH;
      mtFrameMsgBuf->length -= tmp_proLength;
      break;
    default:
      mtFrameMsgBuf->readIdx = (mtFrameMsgBuf->readIdx + 1) % FRAME_MSG_LENGTH;
      mtFrameMsgBuf->length--;
      break;
    }
  }
  return;
}

lidarAPi::DEB_FRAME_S packDEBV3Frame(lidarAPi::SCD_FRAME_TYPE_E flag, int regAddress, int regData)
{
  lidarAPi::DEB_FRAME_S tmpFrame_;
  memset(&tmpFrame_, 0, sizeof(lidarAPi::DEB_FRAME_S));

  switch (flag) {
  case lidarAPi::eCmdWrite:
    tmpFrame_.msgHead = DEB_FRAME_WRITE;
    break;
  case lidarAPi::eCmdRead:
    tmpFrame_.msgHead = DEB_FRAME_READ;
    break;
  case lidarAPi::eCmdQuery:
    tmpFrame_.msgHead = DEB_FRAME_READ;
    break;
  default:
    tmpFrame_.msgHead = DEB_FRAME_WRITE;
    break;
  }
  unsigned short tmpSData = 0;
  tmpSData = regAddress;

  tmpFrame_.regAddress = tmpSData;

  tmpFrame_.regData = regData;

  unsigned char *tmpBase = (unsigned char*)&tmpFrame_;
  tmpFrame_.msgCheckSum = checkSum(tmpBase + 2, sizeof(lidarAPi::DEB_FRAME_S)-2);
  return tmpFrame_;
}
