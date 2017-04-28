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

#include <sstream>
#include <fstream>
#include <time.h>
#include <sys/time.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeCone.h>

using namespace std;
using namespace octomap;

// Function for dumb profiling
double get_wall_time()
{
  struct timeval time;
  gettimeofday(&time, NULL);
  return (double)time.tv_sec + (double)time.tv_usec * 1e-6;
}

int main(int argc, char** argv) {
  // For evaluating computation times
  double tic, toc, dt_seconds;

  cout << endl;
  cout << "Loading tree from file" << endl;

  // Open file
  string fname("simple_tree.bt");
  ifstream infile(fname.c_str());

  // Prepare tree
  OcTreeCone tree (0.1);  // create empty tree with resolution 0.1

  // Load tree
  tic = get_wall_time();
  tree.readBinary(infile);
  toc = get_wall_time();
  dt_seconds = toc - tic;
  printf("Done. %f seconds to load tree from file.\n", dt_seconds);
  cout << endl << endl;

  // Space represented by tree
  double x, y, z;
  tree.getMetricMax(x, y, z);
  double V = x * y * z;
  printf("Total dimension of space: %.2f m^3 (%.1fm x %.1fm x %.1fm)\n\n",
         V, x, y, z);

  // Voxel comparison
  double dv = tree.getResolution();
  printf("Leaf node size = %.2f m\n", dv);
  uint64_t num_voxels = V / pow(dv, 3);

  printf("Num voxels for regular grid with %.2f m resolution: %lu\n", dv, 
         num_voxels);

  // Tree size
  size_t num_nodes = tree.calcNumNodes();
  double percentage = 100.0 * ((double)num_nodes / (double)num_voxels);
  printf("Total number of nodes in tree: %lu (%.2f %%) \n", num_nodes,
         percentage);

  // Memory usage
  double tree_mem = (double)tree.memoryUsage() / 1e6;
  double grid_mem = (double)tree.memoryFullGrid() / 1e6;
  cout << endl << endl;
  printf("OcTree size in memory: %.2f MB\n", tree_mem);
  printf("Full grid size in memory: %.2f MB\n", grid_mem);

  // Approximate tree traversal time
  cout << endl << endl;
  tic = get_wall_time();
  // Set up iterator
  OcTreeCone::leaf_iterator it = tree.begin_leafs();
  OcTreeCone::leaf_iterator end = tree.end_leafs();
  toc = get_wall_time();
  dt_seconds = toc - tic;
  printf("Time to set up iterators: %f s\n", dt_seconds);
  // Set up data containers
  point3d center;
  double cv_prob;
  // Traverse the whole tree
  tic = get_wall_time();
  for(it; it != end; ++it)
  {
    center = it.getCoordinate();
    cv_prob = it->getConeVoxelProbability();
  }
  toc = get_wall_time();
  dt_seconds = toc - tic;
  printf("Time to traverse entire tree = %f\n", dt_seconds);

  return 0;
}
