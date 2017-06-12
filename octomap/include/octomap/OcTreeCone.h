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

#ifndef OCTOMAP_OCTREE_CONE_H
#define OCTOMAP_OCTREE_CONE_H

#include <iostream>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

namespace octomap {

  // forward declaration for "friend"
  class OcTreeCone;
  
  // node definition
  class OcTreeConeNode : public OcTreeNode {    

  public:
    friend class OcTreeCone; // needs access to node children (inherited from AbstractOcTreeNode)
    OcTreeConeNode() : OcTreeNode(), cv_prob(0) {}

    // copy constructor
    OcTreeConeNode(const OcTreeConeNode& rhs) : OcTreeNode(rhs), cv_prob(rhs.cv_prob) {}

    // File I/O
    std::istream& readData(std::istream &s);
    std::ostream& writeData(std::ostream &s) const;

    bool operator==(const OcTreeConeNode& rhs) const{
      return (rhs.value == value && rhs.cv_prob == cv_prob);
    }
    
    void copyData(const OcTreeConeNode& from){
      OcTreeNode::copyData(from);
      this->cv_prob = from.getConeVoxelProbability();
    }
      
    // Cone voxel prob
    inline double getConeVoxelProbability() const { return cv_prob; }
    inline void setConeVoxelProbability(double cv) { this->cv_prob = cv; }
    inline void updateConeVoxelProbability(double cv) { this->cv_prob += cv; }

    inline bool isCVPSet() const { return cv_prob != 0; }

    // Handle children
    void updateCVProbChildren();

    double getAverageChildCVProb() const;

  protected:
    double cv_prob;
  };


  // tree definition
  class OcTreeCone : public OccupancyOcTreeBase <OcTreeConeNode> {    

  public:
    /// Default constructor, sets resolution of leafs
	OcTreeCone(double resolution);
      
    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    OcTreeCone* create() const {return new OcTreeCone(resolution); }

    std::string getTreeType() const {return "OcTreeCone";}

    // For manipulating tree nodes
    virtual bool pruneNode(OcTreeConeNode* node);
    virtual bool isNodeCollapsible(const OcTreeConeNode* node) const;

    // set node color at given key or coordinate
    OcTreeConeNode* setNodeCVProb(const OcTreeKey &key, double cv);
    OcTreeConeNode* setNodeCVProb(float x, float y, float z, double cv)
    {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x, y, z), key)) return NULL;
      return setNodeCVProb(key, cv);
    }

    // NOTE: SKIPPING INTEGRATE/AVERAGE METHODS FROM COLOROCTREE TEMPLATE
    //   THESE METHODS WOULD BE DEFINED HERE IF NEEDED

    // Update inner nodes: set cv_prob to average child cv_prob
    void updateInnerOccupancy();

    /* Math operations to perform on the cv_prob of every node in tree */

    // Scale cv_prob
    void scale(double s);

    // Return the sum of the cone voxel probability over all nodes
    double sum();

    // Return the min/max of the cone-voxel probability over all nodes
    double min();
    double max();

    // Normalize the cvp so that the sum of cvp over all nodes == 1
    void normalize();

  protected:
    void updateInnerOccupancyRecurs(OcTreeConeNode* node, unsigned int depth);
    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a 
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */
    class StaticMemberInitializer{
    public:
      StaticMemberInitializer() {
        OcTreeCone* tree = new OcTreeCone(0.1);
        tree->clearKeyRays();
        AbstractOcTree::registerTreeType(tree);
      }

      /**
      * Dummy function to ensure that MSVC does not drop the
      * StaticMemberInitializer, causing this tree failing to register.
      * Needs to be called from the constructor of this octree.
      */
      void ensureLinking() {};
    };
    /// to ensure static initialization (only once)
    static StaticMemberInitializer ocTreeConeMemberInit;
    
  };

} // end namespace

#endif
