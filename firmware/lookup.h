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
/*   0,     5,  10,  15,  20,  25,  30,  35,  40,  45,  50,  55,  60,*/
/*  25,*/  51,  45,  40,  20,  25,  20,  20,  35,  40,  40,  49,  51,
/*  50,*/ 130,  91,  65,  20,  25,  20,  20,  35,  40,  40,  49,  51,
/*  75,*/ 170, 111,  83,  65,  25,  20,  20,  35,  40,  40,  49,  51,
/* 100,*/ 170, 130,  95,  78,  63,  20,  20,  35,  40,  40,  49,  51,
/* 150,*/ 170, 180, 114,  94,  83,  75,  54,  35,  40,  40,  49,  51,
/* 200,*/ 170, 180, 135, 107,  95,  85,  76,  70,  61,  40,  49,  51,
/* 250,*/ 170, 180, 165, 120, 105,  94,  86,  80,  74,  65,  49,  51,
/* 300,*/ 170, 180, 165, 133, 114, 101,  93,  87,  81,  73,  64,  57,
/* 350,*/ 170, 180, 165, 152, 123, 108,  99,  93,  87,  80,  76,  64,
/* 400,*/ 170, 180, 165, 152, 132, 115, 104,  98,  91,  85,  80,  76,
/* 450,*/ 170, 180, 165, 152, 142, 123, 108, 103,  95,  89,  85,  80,
/* 500,*/ 170, 180, 165, 152, 155, 131, 112, 108,  99,  93,  89,  85
};
