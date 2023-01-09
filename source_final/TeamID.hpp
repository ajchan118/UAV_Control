#pragma once

//We define the Team ID (your team's unique number) in this file.
//This is the unique number assigned to your team, and is used (amongst other
//things) to define the radio communication. You must change this from zero.
//**Only** replace the number 0 with your team's ID. Do not make any other
//changes to this file!

#define ME136_TEAM_ID 42

//----------------------------------------------------------------//
//Don't change the below:
#if ME136_TEAM_ID == 0
#error "You must set your team id!"
#endif
