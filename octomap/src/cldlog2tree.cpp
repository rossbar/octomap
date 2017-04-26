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
#include <ctime>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace std;
using namespace octomap;


void print_query_info(point3d query, OcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  }
  else 
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
}

int main(int argc, char** argv) {
  // For evaluating computation times
  time_t tic, toc;
  double dt_seconds;

  cout << endl;
  cout << "Loading point cloud from file" << endl;

  // Open file
  string fname;
  string line;
  fname = "/home/ross/Desktop/cld.log";
  ifstream infile(fname.c_str());

  // Create point cloud
  Pointcloud* cld = new Pointcloud();
  float x, y, z;

  tic = time(NULL);

  // Read every line in file
  while (true)
  {
    getline(infile, line);
    if( infile.good() && !infile.eof())
    {
      stringstream sline;
      sline << line;
      // Get floats from lines
      sline >> x >> y >> z;
      // Add to pointcloud
      cld->push_back(x, y, z);
    }
    else break;
  }

  // Evaluate time
  toc = time(NULL);
  dt_seconds = difftime(toc, tic);
  printf("Done. %f seconds to load data from file.\n", dt_seconds);
  cout << "Adding scan to octree..." << endl;

  // Prepare tree
  OcTree tree (0.1);  // create empty tree with resolution 0.1
  point3d origin (0, 0, 0);
  // Default args
  double maxrange = -1.0;
  bool lazy_eval = false;
  bool discretize = false;

  tic = time(NULL);
  // Add cloud to tree
  tree.insertPointCloud(cld, origin, maxrange, lazy_eval, discretize);
  toc = time(NULL);
  dt_seconds = difftime(toc, tic);
  printf("Done. %f seconds to add %lu points to tree.\n", dt_seconds, cld->size());

//  // insert some measurements of occupied cells
//
//  for (int x=-20; x<20; x++) {
//    for (int y=-20; y<20; y++) {
//      for (int z=-20; z<20; z++) {
//        point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
//        tree.updateNode(endpoint, true); // integrate 'occupied' measurement
//      }
//    }
//  }
//
//  // insert some measurements of free cells
//
//  for (int x=-30; x<30; x++) {
//    for (int y=-30; y<30; y++) {
//      for (int z=-30; z<30; z++) {
//        point3d endpoint ((float) x*0.02f-1.0f, (float) y*0.02f-1.0f, (float) z*0.02f-1.0f);
//        tree.updateNode(endpoint, false);  // integrate 'free' measurement
//      }
//    }
//  }

  cout << endl;
  cout << "performing some queries:" << endl;
  
  point3d query (0., 0., 0.);
  OcTreeNode* result = tree.search (query);
  print_query_info(query, result);

  query = point3d(-1.,-1.,-1.);
  result = tree.search (query);
  print_query_info(query, result);

  query = point3d(1.,1.,1.);
  result = tree.search (query);
  print_query_info(query, result);


  cout << endl;
  tree.writeBinary("simple_tree.bt");
  cout << "wrote example file simple_tree.bt" << endl << endl;
  cout << "now you can use octovis to visualize: octovis simple_tree.bt"  << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;  

}
