#ifndef _rx_h_included_
#define _rx_h_included_

#include "rxsettings.h"
#include "orangerx.h" // Hardware instance

class IRfm;

class Receiver
{
public:
  Receiver(Hardware* hardware)
  {
		m_hardware = hardware;
			
  }

  void Configure(RxSettings settings)
  {
  }

  void InitRfm(IRfm* rfm)
  {
  }



private:
  IRfm* m_rfm; // Radio frequency module device used by the receiver.
	Hardware *m_hardware;
};

#endif
