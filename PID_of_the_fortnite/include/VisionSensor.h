/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature NEUTRAL = vex::vision::signature (1, 1503, 1965, 1734, -3687, -3371, -3529, 5.5, 0);
vex::vision::signature RED_ALLIANCE = vex::vision::signature (2, 6247, 10413, 8330, -599, 1, -299, 1.9, 0);
vex::vision::signature BLUE_ALLIANCE = vex::vision::signature (3, -2777, -1911, -2344, 6707, 10923, 8815, 2.4, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision AllSeeingEye = vex::vision (vex::PORT5, 50, NEUTRAL, RED_ALLIANCE, BLUE_ALLIANCE, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/