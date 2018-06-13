/**
 *  @file    DA_TCPConfig.h
 *  @author  peter c
 *  @date    05/17/2018
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Hydroponic sensor query config
 *  Not all information/control is implemented via Modbus
 *  A simple command line approach that handles
 *
 *   atlas x Command	Set serial command to atlas sensor on channel x
 *   remote XXX	change atlas port speed
 *   1wire echo	display current 1-wire data
 *   atlas echo	display current Atlas sensor values
 *   atlas suspend | resume	display current atlas data
 *   remote info	general info about remote I/O
 *   help
 *   remote IP xxx	set IP
 *   remote gateway xxx	set gateway
 *   remote subnet xxx	set subnet
 *   remote MAC xxx	set MAC

 *
 *    FUTURE: replace with HTTP interface
 */

 #ifndef DA_TCPCOMMANDHANDLER_H
 #define DA_TCPCOMMANDHANDLER_H
  #include <Ethernet.h>
  #define DA_TCP_LEN 60 // max packet to parse
  #define DA_TCP_PORT 1867

  #define DA_TCP_COMMAND_GROUP_ATLAS 0
  #define DA_TCP_COMMAND_GROUP_REMOTE 1
  #define DA_TCP_COMMAND_GROUP_ONEWIRE 2
  #define DA_TCP_COMMAND_GROUP_HELP 3
  #define DA_TCP_COMMAND_GROUP_COUNT 4

#define DA_TCP_COMMAND_MAX_ARGS 5

struct _remoteIOCommandEntry
{
  char commandGroup[7];
  void (*commandHandler)(uint8_t    argc,
               char **argv, Stream* aOutputStream);
};

typedef _remoteIOCommandEntry remoteIOCommandEntry;

class DA_TCPCommandHandler {
public:

  DA_TCPCommandHandler();
  void refresh();

  char msgByteArray[DA_TCP_LEN]; // send and recieve buffer3:i
  void init();
  bool addCommandHandler(uint8_t   commandIdx,
                         void (   *handler)(
                           uint8_t     argc,
                           char  **argv,
                           Stream *outputStream));

protected:

  bool processCommand( char* aCommand, Stream* aOutputStream);

private:

  remoteIOCommandEntry commandHandlerEntries[DA_TCP_COMMAND_GROUP_COUNT] =
  {{ "atlas", NULL }, { "remote", NULL }, { "1wire", NULL }, { "help", NULL } };

  //  remoteIOCommandEntry commandHandler[3] = {{1,2},{3,4},{5,6}};
};
 #endif // ifndef DA_TCPCOMMANDHANDLER_H
