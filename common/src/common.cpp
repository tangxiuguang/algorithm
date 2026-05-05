#include "../include/common.h"

double manhattan(int x1, int y1, int x2, int y2) {
	return abs(x1 - x2) + (y1 - y2);
}

double chebyshev(int x1, int y1, int x2, int y2) {
	return std::max(abs(x1 - x2), abs(y1 - y2));
}
