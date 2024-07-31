#ifndef KDTREE_SHARED_H
#define KDTREE_SHARED_H

#include "ikd_Tree.h"

typedef pcl::PointXYZI PointType2;

extern KD_TREE<PointType2>::Ptr kdtree_plane_ptr;
extern KD_TREE<PointType2>::Ptr kdtree_line_ptr;

void initializeKDTree();

#endif