#include "Intersection.h"

namespace Geotree
{
    Intersection::Intersection(const Mesh _mesh1, const Mesh _mesh2)
            : mesh1(_mesh1), mesh2(_mesh2)
    {
        std::set <SegmentT> segments;

    }

    uint32_t Intersection::pathcount(void)
    {
        return 1;
    }
}