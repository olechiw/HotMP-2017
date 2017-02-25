#ifndef MotionProfile__h_
#define MotionProfile__h_

#include <vector>

const int kMotionProfileSz =200;

//Select the green highlighted cells and paste into  a csv file.
//No need to copy the blank lines at the bottom.
//This can be pasted into an array for direct use in C++/Java.
//       Position (rotations)	Velocity (RPM)	Duration (ms)
std::vector<std::vector<double>> profileLeft();

std::vector<std::vector<double>> profileRight();





#endif // MotionProfile__h_
