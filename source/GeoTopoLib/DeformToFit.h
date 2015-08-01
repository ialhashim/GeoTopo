#pragma once

namespace Structure{ struct Node; }

class DeformToFit
{
public:
    static void registerAndDeformNodes(Structure::Node *snode, Structure::Node *tnode);
};
