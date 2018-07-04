/**
 *  @file    DA_SCD30.cpp
 *  @author  peter c
 *  @date    7/1/2018
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *
 **/

 #include "DA_SCD30.h"


DA_SDC30::DA_SDC30(Stream& s)  : DA_Input(IO_TYPE::SDC30), serialPort(s)
{}

void DA_SDC30::init()
{
  //serialPort.begin(SCD30_BAUD);


  node.begin(SCD30_ADDRESS, serialPort);
}

void DA_SDC30::serialize(Stream *aOutputStream,
                         bool    includeCR)
{}


void DA_SDC30::onRefresh()
{}

void DA_SDC30::refreshAll()
{}
