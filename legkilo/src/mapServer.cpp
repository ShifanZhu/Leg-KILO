#include "mapServer.h"

KD_TREE<PointType2>::Ptr kdtree_plane_ptr;
KD_TREE<PointType2>::Ptr kdtree_line_ptr;

void initializeKDTree(){
    kdtree_plane_ptr = std::make_shared<KD_TREE<PointType2>>(0.5, 0.6, 0.25);
    kdtree_line_ptr  = std::make_shared<KD_TREE<PointType2>>(0.5, 0.6, 0.1);
}
