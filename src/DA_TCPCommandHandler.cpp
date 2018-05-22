/**
 *  @file    DA_TCPConfig.h
 *  @author  peter c
 *  @date    05/17/2018
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Hydroponic sensor query config
 **/
#include <Streaming.h>
#include "DA_TCPCommandHandler.h"


EthernetServer DA_TCPCommandHandlerServer(DA_TCP_PORT);


DA_TCPCommandHandler::DA_TCPCommandHandler() {}

void DA_TCPCommandHandler::init()
{
  DA_TCPCommandHandlerServer.begin();
}

bool DA_TCPCommandHandler::processCommand(char *aCommand, Stream *aOutputStream)
{
  bool    rv = false;
  char   *argv[DA_TCP_COMMAND_MAX_ARGS];
  char   *commandGroup = strtok(aCommand, " ");
  uint8_t i            = 0;
  uint8_t argc         = 0;

  argv[i] = strtok(NULL, " ");

  while ((i < DA_TCP_COMMAND_MAX_ARGS) && (argv[i] != NULL)) argv[++i] = strtok(
      NULL,
      " ");

  argc = i;

  for (i = 0; i < DA_TCP_COMMAND_GROUP_COUNT; i++)
  {
    if (!strcmp(commandHandlerEntries[i].commandGroup,
                commandGroup))
    {
      if (commandHandlerEntries[i].commandHandler !=
          NULL) commandHandlerEntries[i].commandHandler(argc, argv,
                                                        aOutputStream);
      else *aOutputStream << F("NO handler available:") << i << endl;
      rv = true;
      break;
    }
  }

  return rv;
}

bool DA_TCPCommandHandler::addCommandHandler(uint8_t aCommandIdx,
                                             void (*handler)(
                                               uint8_t argc,
                                               char **argv,
                                               Stream *outputStream))
{
  bool rv = false;

  if ((aCommandIdx >= 0) && (aCommandIdx < DA_TCP_COMMAND_GROUP_COUNT))
  {
    rv                                                = true;
    commandHandlerEntries[aCommandIdx].commandHandler = handler;
  }

  return rv;
}

void DA_TCPCommandHandler::refresh()
{
  EthernetClient client = DA_TCPCommandHandlerServer.available();

  if (client.available())
  {
    // delay(10);
    uint16_t i = 0;
    char     input;

    while (client.available())
    {
      // won't read forever. It will wrap around storage buffer and command
      // will fail downstream
      if (i > (DA_TCP_LEN - 1)) i = 0;
      input = client.read();

      if ((input == '\n') || (input == '\r')) break;

      msgByteArray[i++] = input;
    }
    msgByteArray[i] = 0;

    if (!processCommand(msgByteArray,
                        &client)) client << F("Invalid Command") << endl;
  }
}
