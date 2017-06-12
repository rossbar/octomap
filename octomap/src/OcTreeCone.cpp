/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "octomap/OcTreeCone.h"

namespace octomap {

/* --------------------- NODE IMPLEMENTATION --------------------------------*/

  // File I/O
  std::istream& OcTreeConeNode::readData(std::istream &s)
  {
    s.read((char*) &value,   sizeof(value));   // Read occupancy value
    s.read((char*) &cv_prob, sizeof(cv_prob)); // Read cone voxel prob.
    return s;
  }

  std::ostream& OcTreeConeNode::writeData(std::ostream &s) const
  {
    s.write((const char*) &value,   sizeof(value));   // Write occupancy
    s.write((const char*) &cv_prob, sizeof(cv_prob)); // Write cv probability
    return s;
  }

  double OcTreeConeNode::getAverageChildCVProb() const
  {
    double cvsum = 0;
    int c = 0;
    if (children != NULL)
    {
      for(int i = 0; i<8; i++)
      {
        OcTreeConeNode* child = static_cast<OcTreeConeNode*>(children[i]);
        if(child != NULL && child->isCVPSet())
        { 
          cvsum += child->getConeVoxelProbability();
          ++c;
        }
      }
    }
    if (c > 0)
    {
      cvsum /= c;
      return cvsum;
    }
    else { return 0; }
  }

  void OcTreeConeNode::updateCVProbChildren()
  {
    cv_prob = getAverageChildCVProb();
  }

/* ---------------------- TREE IMPLEMENTATION -------------------------------*/

  OcTreeCone::OcTreeCone(double resolution)
   : OccupancyOcTreeBase<OcTreeConeNode>(resolution) {
    ocTreeConeMemberInit.ensureLinking();
  }

  OcTreeConeNode* OcTreeCone::setNodeCVProb(const OcTreeKey &key, double cv)
  {
    OcTreeConeNode* n = search(key);
    if(n != 0)
    {
      n->setConeVoxelProbability(cv);
    }
    return n;
  }

  bool OcTreeCone::pruneNode(OcTreeConeNode* node)
  {
    // Not collapsible
    if(!isNodeCollapsible(node)) return false;
    // If collapsible, set cv prob
    node->copyData(*(getNodeChild(node, 0)));
    if(node->isCVPSet())
      node->setConeVoxelProbability(node->getAverageChildCVProb());
    // delete children
    for (unsigned int i = 0; i < 8; i++)
    {
      deleteNodeChild(node, i);
    }
    delete[] node->children;
    node->children = NULL;
    return true;
  }

  bool OcTreeCone::isNodeCollapsible(const OcTreeConeNode* node) const
  {
    // Conditions: 1) all children exists, 2) children don't have children of
    // their own, 3) children have same occupancy probability
    if(!nodeChildExists(node, 0)) return false;

    const OcTreeConeNode* firstChild = getNodeChild(node, 0);
    if(nodeHasChildren(firstChild)) return false;

    for(unsigned int i = 1; i < 8; i++)
    {
      // Compare nodes using only occupancy; ignore cv_prob for pruning
      if(!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) ||
         !(getNodeChild(node, i)->getValue() == firstChild->getValue()))
        return false;
    }
    return true;
  }

  void OcTreeCone::updateInnerOccupancy()
  {
    this->updateInnerOccupancyRecurs(this->root, 0);
  }

  void OcTreeCone::updateInnerOccupancyRecurs(OcTreeConeNode* node, unsigned int depth)
  {
    // only recurse and update for inner nodes
    if(nodeHasChildren(node))
    {
      // return early for last level
      if (depth < this->tree_depth)
      {
        for(unsigned int i = 0; i < 8; i++)
        {
          if(nodeChildExists(node, i))
          {
            updateInnerOccupancyRecurs(getNodeChild(node, i), depth+1);
          }
        }
      }
      node->updateOccupancyChildren();
      node->updateCVProbChildren();
    }
  }

  // Multiply cv_prob by a scalar value
  void OcTreeCone::scale(double s)
  {
    for(leaf_iterator it = this->begin_leafs(), end=this->end_leafs();
        it != end; ++it)
    {
      it->setConeVoxelProbability(s * it->getConeVoxelProbability());
    }
  }

  double OcTreeCone::sum()
  {
    double cvsum = 0;
    for(leaf_iterator it = this->begin_leafs(), end=this->end_leafs(); 
        it != end; ++it)
    {
      cvsum += it->getConeVoxelProbability();
    }
    return cvsum;
  }

  double OcTreeCone::max()
  {
    double cv;
    double cvmax = 0;
    for(leaf_iterator it = this->begin_leafs(), end=this->end_leafs();
        it != end; ++it)
    {
      cv = it->getConeVoxelProbability();
      if(cv > cvmax){ cvmax = cv; }
    }
    return cvmax;
  }
    
  double OcTreeCone::min()
  {
    double cv;
    double cvmin = 0;
    for(leaf_iterator it = this->begin_leafs(), end=this->end_leafs();
        it != end; ++it)
    {
      cv = it->getConeVoxelProbability();
      if(cv < cvmin){ cvmin = cv; }
    }
    return cvmin;
  }

  void OcTreeCone::normalize()
  {
    double cvsum = this->sum();
    for(leaf_iterator it = this->begin_leafs(), end=this->end_leafs();
        it != end; ++it)
    {
      it->setConeVoxelProbability(it->getConeVoxelProbability() / cvsum);
    }
    return;
  }

  OcTreeCone::StaticMemberInitializer OcTreeCone::ocTreeConeMemberInit;

} // end namespace

