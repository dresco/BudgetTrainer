#define SPEED_COLS 12
#define POWER_ROWS 12

#define max(a,b)    (((a) > (b)) ? (a) : (b))
#define min(a,b)    (((a) < (b)) ? (a) : (b))

uint16_t power_index[POWER_ROWS] = {25,50,75,100,150,200,250,300,350,400,450,500};

uint16_t speed_index[SPEED_COLS] = {5,10,15,20,25,30,35,40,45,50,55,60};

//
// Encoded resistance values.
// Add +50 to the actual 1-100 values, so that we can maintain the
// correct slope at the limits and remain within a uint8_t datatype
//
// Edge values have been estimated to maintain a vaguely correct slope
// at the limits of resistance adjustment.
//
// todo: refine values
//
uint8_t lookup_table_1d[POWER_ROWS * SPEED_COLS] = {
 /*  0,     5,  10,  15,  20,  25,  30,  35,  40,  45,  50,  55,  60,*/
 /* 25,*/  94,  25,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
 /* 50,*/ 106,  90,  52,  25,   1,   1,   1,   1,   1,   1,   1,   1,
 /* 75,*/ 120, 109,  83,  51,  25,   1,   1,   1,   1,   1,   1,   1,
 /*100,*/ 150, 127,  94,  78,  55,  31,  31,   1,   1,   1,   1,   1,
 /*150,*/ 200, 177, 113,  94,  83,  76,  58,  45,  45,   1,   1,   1,
 /*200,*/ 200, 200, 133, 106,  96,  86,  76,  70,  57,  49,   1,   1,
 /*250,*/ 200, 200, 170, 119, 104,  95,  87,  78,  72,  62,  50,  51,
 /*300,*/ 200, 200, 200, 133, 113, 101,  94,  85,  80,  71,  63,  52,
 /*350,*/ 200, 200, 200, 153, 123, 109, 100,  91,  87,  79,  70,  62,
 /*400,*/ 200, 200, 200, 200, 132, 115, 106,  97,  92,  86,  79,  70,
 /*450,*/ 200, 200, 200, 200, 144, 124, 110, 104,  96,  92,  85,  73,
 /*500,*/ 200, 200, 200, 200, 156, 133, 114, 111, 100,  98,  92,  85
};



/*
uint8_t lookup_table_2d[POWER_ROWS][SPEED_COLS] = {
{44,    1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1},
{66,    40,     2,      1,      1,      1,      1,      1,      1,      1,      1,      1},
{100,   59,     33,     1,      1,      1,      1,      1,      1,      1,      1,      1},
{100,   77,     44,     28,     5,      1,      1,      1,      1,      1,      1,      1},
{100,   100,    63,     44,     33,     26,     8,      1,      1,      1,      1,      1},
{100,   100,    83,     56,     46,     36,     26,     20,     7,      1,      1,      1},
{100,   100,    100,    69,     54,     45,     37,     28,     22,     12,     1,      1},
{100,   100,    100,    83,     63,     51,     44,     35,     30,     21,     13,     1},
{100,   100,    100,    100,    73,     59,     50,     41,     37,     29,     20,     12},
{100,   100,    100,    100,    82,     65,     56,     47,     42,     36,     29,     20},
{100,   100,    100,    100,    94,     74,     60,     54,     46,     42,     35,     23},
{100,   100,    100,    100,    100,    83,     64,     61,     50,     48,     42,     35}
};
*/

