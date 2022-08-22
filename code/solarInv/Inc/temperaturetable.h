// Thermistor lookup table for RepRap Temperature Sensor Boards (http://make.rrrf.org/ts)
// Made with createTemperatureLookup.py (http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py)
// ./createTemperatureLookup.py --r0=10000 --t0=25 --r1=99999999 --r2=10000 --beta=3988 --max-adc=4093
// r0: 10000
// t0: 25
// r1: 99999999
// r2: 10000
// beta: 3988
// max adc: 4093
// multiplier: 4093
#define NUMTEMPS 20
short temptable[NUMTEMPS][2] = {
   {1, 515},
   {216, 107},
   {431, 81},
   {646, 67},
   {861, 57},
   {1076, 49},
   {1291, 43},
   {1506, 37},
   {1721, 32},
   {1936, 27},
   {2151, 22},
   {2366, 18},
   {2581, 13},
   {2796, 8},
   {3011, 3},
   {3226, -1},
   {3441, -7},
   {3656, -15},
   {3871, -27},
   {4086, -71}
};
