#ifndef GICP_H
#define GICP_H

#include "AbstractIcp.h"

class Gicp : public AbstractIcp {
public:
    Gicp();

    AbstractIcp& align(PointCloudT& src, PointCloudT& tgt) override;
};
#endif // GICP_H
