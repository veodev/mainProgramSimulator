#include "direction.h"


Direction directionByProbe(int value)
{
    Direction dir = UnknownDirection;
    if (value < 0) {
        dir = BackwardDirection;
    }
    else {
        dir = ForwardDirection;
    }

    return dir;
}

Direction directionByEChannelDir(eChannelDir value)
{
    Direction dir = UnknownDirection;

    switch (value) {
    case cdZoomIn:
        dir = ForwardDirection;
        break;
    case cdZoomOut:
        dir = BackwardDirection;
        break;
    case cdNone:
        dir = UnknownDirection;
        break;
    default:
        break;
    }

    return dir;
}
