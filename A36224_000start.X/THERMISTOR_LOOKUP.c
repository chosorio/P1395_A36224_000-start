
static const int lookup[256]=
{
    23300,23300,23300,23300,23300,23300,23514,23725,23923,24100,24260,24408,24546,24678,24800,24914,25027,25133,25238,25336,25428,25517,25605,25689,25774,25853,25929,26005,26077,26150,26222,26288,26354,26421,26488,26551,26611,26673,26734,26793,26850,26907,26964,27021,27076,27130,27184,27236,27287,27339,27390,27441,27491,27541,27591,27640,27688,27736,27785,27833,27879,27926,27974,28020,28066,28111,28155,28200,28246,28291,28337,28381,28424,28467,28511,28556,28600,28644,28687,28730,28773,28816,28860,28903,28945,28987,29029,29073,29116,29160,29203,29245,29287,29330,29373,29416,29458,29500,29543,29586,29630,29673,29716,29759,29803,29847,29892,29935,29978,30022,30067,30111,30155,30200,30245,30291,30337,30383,30428,30474,30520,30567,30615,30662,30709,30756,30803,30851,30900,30950,31000,31050,31100,31149,31200,31253,31307,31358,31410,31466,31520,31573,31628,31686,31743,31800,31857,31915,31974,32034,32096,32158,32220,32284,32350,32417,32483,32550,32617,32687,32759,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767,32767
};
//Temperatures in Kelvin

unsigned int ConvertDigitalToTemp(unsigned int digital_in){
    //Hopefully the compiler is good enough to optimize this out. 
    //Convert 16 bit to 8 bit.
    unsigned int lookup_index=digital_in>>8;
    //use lookup table
    int base=lookup[lookup_index];
    int next=lookup[lookup_index+1];
    //interpolate data
    int numerator= digital_in&&0xF;
    int denominator=0x10;
    //Doing the ratio this way should be fine, as we have a 32 bit ALU.
    int adder=(next-base)*numerator/denominator;
    return base+adder;


}