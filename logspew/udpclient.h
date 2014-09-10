/**
 * \file
 * Brief description. Longer description.
 * 
 * \author $Author$
 * \date $Date$
 */


#ifndef __UDPCLIENT_H
#define __UDPCLIENT_H

void udpInit();
bool udpSend(const char *host, int port,const char *msg);

#endif /* __UDPCLIENT_H */
