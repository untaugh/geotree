#pragma once

const double NEAR_ZERO = 1.0e-14;

#define NEAREQUAL(a,b) ((a - b) < NEAR_ZERO && (a - b) > -NEAR_ZERO)
#define NEARZERO(a) (a < NEAR_ZERO && a > -NEAR_ZERO)
