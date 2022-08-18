G01 X30 Y30 Z8 F500; Go to origin+4mm and set rate to 8mm/min
G04 P10000; hold for 1s
G01 X30 Y30 Z2.55 ; Load at center
G04 P10000; 
G01 X30 Y30 Z8 ;Unload
G04 P10000; 
G01 X30 Y60 Z8; Move to top 
G01 X30 Y60 Z2.55 ;Load at top
G04 P10000; 
G01 X30 Y60 Z8 ;Unload
G04 P10000; 
G01 X60 Y30 Z8; Move to right
G01 X60 Y30 Z2.55 ;Load at right
G04 P10000; 
G01 X60 Y30 Z8 ;Unload
G04 P10000; 
G01 X15 Y15 Z8; Move to mid-bot-left
G01 X15 Y15 Z2.55 ;Load at mid-bot-left
G04 P10000; 
G01 X15 Y15 Z8; Unload
G04 P10000; 
G01 X30 Y30 Z8; Go to origin
