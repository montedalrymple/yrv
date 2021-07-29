# ##############################################################################

# iCEcube SDC

# Version:            2017.08.27940

# File Generated:     Jan 1 2021 14:24:58

# ##############################################################################

####---- CreateClock list ----2
create_clock  -period 10.00 -name {MHZ_100} [get_ports {MHZ_100}] 
create_clock  -period 50.00 -name {clk} [get_nets {clk}] 

