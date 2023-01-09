#pragma once
/* Do not modify this header file.
 */

#include "MainLoopTypes.hpp"

extern "C" {  // <- needed to make functions visible to library we're compiling
/* Main loop function
 *
 * This is where the action happens. This is run at 500Hz, i.e. every 2ms.
 * The input is a list of sensor measurements, etc., and the outputs are the
 * desired motor commands and the states of the LEDs.
 */
MainLoopOutput MainLoop(MainLoopInput const &in);

/* A debugging function, that we can call from the shell, to print information
 * about the system state.
 */
void PrintStatus();

}
