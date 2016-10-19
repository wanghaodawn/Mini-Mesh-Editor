/*
 * Student solution for CMU 15-462 Project 2 (MeshEdit)
 *
 * Implemented by Hao Wang on 10/09/2015.
 *
 */

#include "student_code.h"
#include "mutablePriorityQueue.h"

//#define ENABLE_DEBUG
#ifdef ENABLE_DEBUG
#define DEBUG_CODE(CODEFRAGMENT) CODEFRAGMENT;
#else
#define DEBUG_CODE(CODEFRAGMENT)
#endif

namespace CMU462
{
   // Compute the degree of a given vertex
   int computeDegree(VertexIter v) {
      int degree = 0;
      HalfedgeIter halfedge = v->halfedge();
      do {
         halfedge = halfedge->twin()->next();
         degree++;
      } while (halfedge != v->halfedge());

      return degree;
   }

   bool isCollapsable(EdgeIter e) {

      HalfedgeIter h1 = e->halfedge()->next()->next()->twin()->next()->next()->twin();
      HalfedgeIter h2 = e->halfedge()->twin()->next()->next()->twin()->next()->next()->twin();
      HalfedgeIter hf2 = h2;
      HalfedgeIter end1 = e->halfedge()->twin()->next();
      HalfedgeIter end2 = e->halfedge()->next();

      while (h1 != end1) {
         h2 = hf2;
         while (h2 != end2)
         {
            if (h1->twin()->vertex() == h2->twin()->vertex()) {
               return false;
            }
            h2 = h2->next()->next()->twin();
         }
         h1 = h1->next()->next()->twin();
      }

      VertexIter v1 = e->halfedge()->next()->next()->vertex();
      VertexIter v2 = e->halfedge()->twin()->next()->next()->vertex();
      if ((!v1->isBoundary() && v1->degree() == 3) || \
          (!v2->isBoundary() && v2->degree() == 3))
         return false;

      return true;
   }

   void printDetails (HalfedgeIter xy) {
      cout<< "vertex: "<< xy->vertex()->position << endl;
      cout<< "length: "<< xy->edge()->length()<< endl;
      cout<< "next: " << &(*(xy->next())) << endl;
      cout<< "twin: " << &(*(xy->twin())) << endl;
      cout<< "face: " << &(*(xy->face())) << endl;
      cout<< "edge: " << &(*(xy->edge())) << endl;
      cout<<endl;
   }

   VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
   {
      // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
      // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      if (e0->isBoundary() || e0->halfedge()->isBoundary() || \
          e0->halfedge()->twin()->isBoundary())
         return verticesEnd();
      // DEBUG_CODE(cout<< "isBoundary:" << e0->isBoundary()<<endl);

      // Get abc
      HalfedgeIter bc = e0->halfedge();
      HalfedgeIter ca = bc->next();
      HalfedgeIter ab = ca->next();

      // Get bdc
      HalfedgeIter cb = bc->twin();
      HalfedgeIter bd = cb->next();
      HalfedgeIter dc = bd->next();

      // Get vertexes
      VertexIter a = ab->vertex();
      VertexIter b = bd->vertex();
      VertexIter c = ca->vertex();
      VertexIter d = dc->vertex();

      // Compute m
      VertexIter m = newVertex();
      m->position = Vector3D((b->position + c->position) / 2.f);

      // New HalfedgeIter
      HalfedgeIter am = newHalfedge();
      HalfedgeIter bm = newHalfedge();
      // HalfedgeIter cm = cb;
      HalfedgeIter dm = newHalfedge();
      HalfedgeIter ma = newHalfedge();
      HalfedgeIter mb = newHalfedge();
      // HalfedgeIter mc = bc;
      HalfedgeIter md = newHalfedge();

      // New EdgeIter
      EdgeIter am_edge = newEdge();
      EdgeIter bm_edge = newEdge();
      EdgeIter cm_edge = bc->edge();
      EdgeIter dm_edge = newEdge();

      // New faces
      FaceIter face0 = ca->face();
      FaceIter face1 = dc->face();
      FaceIter face2 = newFace();
      FaceIter face3 = newFace();

      // Set vertexes
      am->vertex() = a;
      bm->vertex() = b;
      cb->vertex() = c;
      dm->vertex() = d;
      ma->vertex() = m;
      mb->vertex() = m;
      bc->vertex() = m;
      md->vertex() = m;
      
      // next, twin, vertex, edge, face
      // Set acm
      am->setNeighbors(bc, ma, a, am_edge, face0);
      bc->setNeighbors(ca, cb, m, bc->edge(), face0);
      ca->setNeighbors(am, ca->twin(), c, ca->edge(), face0);

      // Set cdm
      cb->setNeighbors(md, bc, c, cb->edge(), face1);
      md->setNeighbors(dc, dm, m, dm_edge, face1);
      dc->setNeighbors(cb, dc->twin(), d, dc->edge(), face1);

      // Set abm
      bm->setNeighbors(ma, mb, b, bm_edge, face2);
      ma->setNeighbors(ab, am, m, am_edge, face2);
      ab->setNeighbors(bm, ab->twin(), a, ab->edge(), face2);

      // Set bdm
      dm->setNeighbors(mb, md, d, dm_edge, face3);
      mb->setNeighbors(bd, bm, m, bm_edge, face3);
      bd->setNeighbors(dm, bd->twin(), b, bd->edge(), face3);

      // Set vertices
      a->halfedge() = ab;
      b->halfedge() = bd;
      c->halfedge() = ca;
      d->halfedge() = dc;
      m->halfedge() = bc;

      // Set for upsampling
      m->isNew = true;

      // Set edges
      am_edge->halfedge() = am;
      bm_edge->halfedge() = bm;
      cm_edge->halfedge() = cb;
      dm_edge->halfedge() = dm;

      // Set for upsampling
      am_edge->isNew = true;
      bm_edge->isNew = true;
      dm_edge->isNew = true;

      // Set faces
      face0->halfedge() = ca;
      face1->halfedge() = dc;
      face2->halfedge() = ab;
      face3->halfedge() = bd;

      return bc->vertex(); 
      }

   VertexIter HalfedgeMesh::collapseEdge( EdgeIter e0 )
   {
      // TODO This method should collapse the given edge and return an iterator to the new vertex created by the collapse.
      if (e0->isBoundary() || e0->halfedge()->isBoundary() || \
          e0->halfedge()->twin()->isBoundary() || !isCollapsable(e0))
         return verticesEnd();
      // DEBUG_CODE(cout<< "isBoundary:" << e0->isBoundary()<<endl;)
       
      // Get halfedges
      HalfedgeIter bc = e0->halfedge();
      HalfedgeIter ca = bc->next();
      HalfedgeIter ab = ca->next();

      HalfedgeIter cb = bc->twin();
      HalfedgeIter bd = cb->next();
      HalfedgeIter dc = bd->next();

      HalfedgeIter ba = ab->twin();
      HalfedgeIter ac = ca->twin();
      HalfedgeIter db = bd->twin();
      HalfedgeIter cd = dc->twin();
       
      // Get vertices 
      VertexIter a = ac->vertex();
      VertexIter b = bd->vertex();
      VertexIter c = ca->vertex();
      VertexIter d = db->vertex();
      
      // Get edges
      EdgeIter bc_edge = e0;
      EdgeIter ab_edge = ab->edge();
      EdgeIter ca_edge = ca->edge();
      EdgeIter dc_edge = dc->edge();
      EdgeIter bd_edge = bd->edge();

      // Get faces 
      FaceIter face0 = bc->face();
      FaceIter face1 = cb->face();

      // Get new elements
      HalfedgeIter md = cd;
      HalfedgeIter ma = ba;
      HalfedgeIter am = ac;
      HalfedgeIter dm = db;
      
      // Set m's new position
      VertexIter m = b;
      m->position = (c->position + b->position) / 2;

      EdgeIter md_edge = bd_edge;
      EdgeIter am_edge = ab_edge;

      // Change the vertices
      HalfedgeIter h  = c->halfedge();
      HalfedgeIter h0 = h;
      do {
         h->vertex() = m;
         h = h->twin()->next();
      } while (h != h0);
      
      // Delete halfedges
      deleteHalfedge(bc);
      deleteHalfedge(ca);
      deleteHalfedge(ab);
      deleteHalfedge(bd);
      deleteHalfedge(dc);
      deleteHalfedge(cb);

      // Delete vertices
      deleteVertex(c);
      
      // Delete edges
      deleteEdge(ca_edge);
      deleteEdge(dc_edge);
      deleteEdge(bc_edge);

      // Delete faces
      deleteFace(face0);
      deleteFace(face1);

      // Set ma
      ma->twin() = am;
      ma->edge() = am_edge;
      ma->vertex() = m;

      // Set md
      md->twin() = dm;
      md->edge() = md_edge;
      md->vertex() = m;

      // Set am
      am->twin() = ma;
      am->edge() = am_edge;

      // Set dm
      dm->twin() = md;
      dm->edge() = md_edge;

      // Set vertices to halfedges
      a->halfedge() = am;
      m->halfedge() = ma;
      d->halfedge() = dm;
      
      // set edges to halfedges
      am_edge->halfedge() = ma;
      md_edge->halfedge() = md;
       
      return m;
   }

   EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
   {
      // TODO This method should flip the given edge and return an iterator to the flipped edge.
      if (e0->isBoundary() || e0->halfedge()->isBoundary() || \
          e0->halfedge()->twin()->isBoundary())
         return edgesEnd();
      // cout<< "isBoundary:" << e0->isBoundary()<<endl;

      // Get abc
      HalfedgeIter bc = e0->halfedge();
      HalfedgeIter ca = bc->next();
      HalfedgeIter ab = ca->next();

      // Get bcd
      HalfedgeIter cb = bc->twin();
      HalfedgeIter bd = cb->next();
      HalfedgeIter dc = bd->next();

      // Get vertexes
      VertexIter a = ab->vertex();
      VertexIter b = bd->vertex();
      VertexIter c = ca->vertex();
      VertexIter d = dc->vertex();

      // Whether it will damage the manifold
      if (b->degree() == 3 || c->degree() == 3)
         return edgesEnd();

      // Get edges
      EdgeIter ab_edge = ab->edge();
      EdgeIter bd_edge = bd->edge();
      EdgeIter dc_edge = dc->edge();
      EdgeIter ca_edge = ca->edge();
      EdgeIter bc_edge = bc->edge();

      // Get faces
      FaceIter f0 = bc->face();
      FaceIter f1 = cb->face();

      // Set acd
      bc->setNeighbors(dc, cb, a, bc_edge, f0);
      dc->setNeighbors(ca, dc->twin(), d, dc_edge, f0);
      ca->setNeighbors(bc, ca->twin(), c, ca_edge, f0);
      
      // Set abd
      cb->setNeighbors(ab, bc, d, bc_edge, f1);
      ab->setNeighbors(bd, ab->twin(), a, ab_edge, f1);
      bd->setNeighbors(cb, bd->twin(), b, bd_edge, f1);

      // Set vertexes
      a->halfedge() = ab;
      b->halfedge() = bd;
      c->halfedge() = ca;
      d->halfedge() = dc;

      ab_edge->halfedge() = ab;
      bd_edge->halfedge() = bd;
      dc_edge->halfedge() = dc;
      ca_edge->halfedge() = ca;
      bc_edge->halfedge() = bc;

      f0->halfedge() = ca;
      f1->halfedge() = bd;

      return bc->edge();
   }

   void MeshResampler::upsample( HalfedgeMesh& mesh )
   // This routine should increase the number of triangles in the mesh using Loop subdivision.
   {
      // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
      // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
      // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
      // the new subdivided e(fine) mesh, which has more elements to traverse.  We will then assign vertex positions in
      // the new mesh based on the values we computed for the original mesh.

      // This step is included in the next step


      // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
      // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
      // TODO a vertex of the original mesh.

      // Compute new positions for all vertices
      VertexIter v = mesh.verticesBegin();
      while (v != mesh.verticesEnd()) {
         // Get the next edge NOW!
         VertexIter nextVertex = v;
         nextVertex++;

         // Initialization
         v->isNew = false;
         v->newPosition = Vector3D(0, 0, 0);
         
         // Get the degree and weight
         int degree = computeDegree(v);

         HalfedgeIter halfedge = v->halfedge();
         HalfedgeIter temp = halfedge;
         
         if (degree == 3) {
            float weight = 3.f/16;
            
            // Compute the new position
            do {
               v->newPosition += temp->twin()->vertex()->position * weight;
               temp = temp->twin()->next();
            } while (temp != halfedge);
            v->newPosition += v->position * 7.f/16;

         } else {
            float weight = 3.f/(8 * degree);

            // Compute the new position
            do {
               v->newPosition += temp->twin()->vertex()->position * weight;
               temp = temp->twin()->next();
            } while (temp != halfedge);
            v->newPosition += v->position * (5.f/8);
         }

         // Update v
         v = nextVertex; 
      }
      
      DEBUG_CODE(cout<<1111<<endl;)
      
      // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
      // Compute updated positions for all edges
      EdgeIter e = mesh.edgesBegin();
      while (e != mesh.edgesEnd()) {
         // Get the next edge NOW!
         EdgeIter nextEdge = e;
         nextEdge++;

         // Initialization
         e->isNew = false;

         VertexIter a = e->halfedge()->vertex();
         VertexIter b = e->halfedge()->next()->vertex();
         VertexIter c = e->halfedge()->next()->next()->vertex();
         VertexIter d = e->halfedge()->twin()->next()->next()->vertex();

         e->newPosition = Vector3D(0, 0, 0);
         e->newPosition  = (3.f/8) * (a->position + b->position);
         e->newPosition += (1.f/8) * (c->position + d->position);
         
         // Update e
         e = nextEdge;
      }
      DEBUG_CODE(cout<<2222<<endl;)

      // TODO Next, we're going to split every edge in the mesh, in any order.  For future
      // TODO reference, we're also going to store some information about which subdivided
      // TODO edges come from splitting an edge in the original mesh, and which edges are new,
      // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
      // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
      // TODO just split (and the loop will never end!)
      
      // Split now
      e = mesh.edgesBegin();
      while (e != mesh.edgesEnd()) {
         // Get the next edge NOW!
         EdgeIter nextEdge = e;
         nextEdge++;

         // Split only when it is not new
         if (e->isNew)
            break;
         else {
            // Set the edge and vertex ->isNew in splitEdge
            VertexIter v1;
            v1 = mesh.splitEdge(e);
            v1->newPosition = v1->halfedge()->edge()->newPosition;
            
            v1->isNew = true;
            v1->halfedge()->next()->next()->edge()->isNew = true;
            v1->halfedge()->twin()->next()->edge()->isNew = true;
            v1->halfedge()->twin()->next()->twin()->next()->edge()->isNew = false;
         }
         // Update e
         e = nextEdge;
      }
      DEBUG_CODE(cout<<3333<<endl;)

      // TODO Now flip any new edge that connects an old and new vertex.
      e = mesh.edgesBegin();
      while (e != mesh.edgesEnd()) {
         // Get the next edge NOW!
         EdgeIter nextEdge = e;
         nextEdge++;

         // Flip only new edges
         if (e->isNew) {
            bool test1 = e->halfedge()->vertex()->isNew;
            bool test2 = e->halfedge()->twin()->vertex()->isNew;
            if ((test1 && !test2) || (!test1 && test2)) {
               mesh.flipEdge(e);
            }
         }
         // Update e
         e = nextEdge;
      }
      DEBUG_CODE(cout<<4444<<endl;)

      // TODO Finally, copy the new vertex positions into final Vertex::position.
      v = mesh.verticesBegin();
      while (v != mesh.verticesEnd()) {
         // Get the next edge NOW!
         VertexIter nextVertex = v;
         nextVertex++;

         v->position = v->newPosition;
         v->newPosition = Vector3D(0, 0, 0);

         v = nextVertex;
      }
      DEBUG_CODE(cout<<5555<<endl;)
      DEBUG_CODE(cout<<endl;)
   }

   // Given an edge, the constructor for EdgeRecord finds the
   // optimal point associated with the edge's current quadric,
   // and assigns this edge a cost based on how much quadric
   // error is observed at this optimal point.
   EdgeRecord::EdgeRecord( EdgeIter& _edge )
   : edge( _edge )
   {
      // TODO Compute the combined quadric from the edge endpoints.
      Matrix4x4 K = edge->halfedge()->vertex()->quadric;
      K = K + edge->halfedge()->twin()->vertex()->quadric;

      // TODO Build the 3x3 linear system whose solution minimizes
      // the quadric error associated with these two endpoints.
      Matrix3x3 A; 
      for (int i = 0; i < 3; i++) {
         for (int j = 0; j < 3; j++) {
            A(i, j) = K(i, j);
         }
      }
      // TODO Use this system to solve for the optimal position, and
      // TODO store it in EdgeRecord::optimalPoint.
      Vector3D b = -K.column(3).to3D();
      optimalPoint = A.inv()*b; 

      // TODO Also store the cost associated with collapsing this edge
      // TODO in EdgeRecord::Cost.
      Vector4D x(optimalPoint.x, optimalPoint.y, optimalPoint.z, 1);
      score = dot(x, K*x);
   }

   void MeshResampler::downsample( HalfedgeMesh& mesh )
   {
      // TODO Compute initial quadrics for each face by simply writing the plane
      // equation for the face in homogeneous coordinates.  These quadrics should
      // be stored in Face::quadric

      FaceIter f = mesh.facesBegin();
      while (f != mesh.facesEnd()) {
         // Get the next edge NOW!
         FaceIter nextFace = f;
         nextFace++;

         Vector3D p = f->halfedge()->vertex()->position;
         Vector3D n = f->normal();
         // Set homogeneous space
         Vector4D t(n.x, n.y, n.z, -dot(n, p));

         f->quadric = outer(t, t);

         f = nextFace;
      }
      DEBUG_CODE(cout<<111111<<endl;)

      // TODO Compute an initial quadric for each vertex as the sum of the quadrics
      // associated with the incident faces, storing it in Vertex::quadric

      VertexIter v = mesh.verticesBegin();
      while (v != mesh.verticesEnd()) {
         // Get the next edge NOW!
         VertexIter nextVertex = v;
         nextVertex++;

         HalfedgeIter h = v->halfedge();
         HalfedgeIter h0 = h;
         v->quadric.zero();

         do {
            // Boundary loops are vain
            if (!h->face()->isBoundary()) {
               v->quadric += h->face()->quadric;
            }
            h = h->twin()->next();
         } while (h != h0);

         v = nextVertex;
      }
      
      DEBUG_CODE(cout<<222222<<endl;)

      // TODO Build a priority queue of edges according to their quadric error cost,
      // TODO i.e., by building an EdgeRecord for each edge and sticking it in the queue.
      
      // Use mutable priority queue to store edge records
      MutablePriorityQueue<EdgeRecord> edgeQueue;
      EdgeIter e = mesh.edgesBegin();
      while (e != mesh.edgesEnd()) {
         // Get the next edge NOW!
         EdgeIter nextEdge = e;
         nextEdge++;

         e->record = EdgeRecord(e);
         edgeQueue.insert(e->record);

         // Update next edge
         e = nextEdge;
      }
      DEBUG_CODE(cout<<333333<<endl;)

      // TODO Until we reach the target edge budget, collapse the best edge.  Remember
      // TODO to remove from the queue any edge that touches the collapsing edge BEFORE
      // TODO it gets collapsed, and add back into the queue any edge touching the collapsed
      // TODO vertex AFTER it's been collapsed.  Also remember to assign a quadric to the
      // TODO collapsed vertex, and to pop the collapsed edge off the top of the queue.

      int edge_num = 6;
      if ((int)mesh.nEdges()/4 > edge_num)
         edge_num = (int)mesh.nEdges()/4;

      while (edgeQueue.queue.size() > 0 && edge_num < mesh.nEdges()) {
         e = edgeQueue.top().edge;
         edgeQueue.pop();

         // Compute new quadric
         Matrix4x4 newQuadric = e->halfedge()->twin()->vertex()->quadric + e->halfedge()->vertex()->quadric;

         // Remove touched edges out of edgeQueue
         HalfedgeIter temp_edge = e->halfedge()->twin()->next();
         HalfedgeIter h0 = e->halfedge();
         while (!temp_edge->edge()->isBoundary() && temp_edge != h0) {
            edgeQueue.remove(temp_edge->edge()->record);
            temp_edge = temp_edge->twin()->next();
         }

         temp_edge = e->halfedge()->next();
         h0 = h0->twin();
         while (!temp_edge->edge()->isBoundary() && temp_edge != h0) {
            edgeQueue.remove(temp_edge->edge()->record);
            temp_edge = temp_edge->twin()->next();
         }

         VertexIter v = mesh.collapseEdge(e);

         if (v != mesh.verticesEnd()) {
            // Collapse Succeed
            v->quadric = newQuadric;

            temp_edge = v->halfedge();
            h0 = temp_edge;
            do {
               if (isCollapsable(temp_edge->edge())) {
                  temp_edge->edge()->record = EdgeRecord(temp_edge->edge());
                  edgeQueue.insert(temp_edge->edge()->record);
               }
               temp_edge = temp_edge->twin()->next();

            } while (!(temp_edge->isBoundary() || temp_edge->twin()->isBoundary() ||\
                     temp_edge->edge()->isBoundary() || temp_edge == h0));

         } else {
            // Collapse failed
            break;
         }
      }
      DEBUG_CODE(cout<<444444<<endl;)
   }

   void Vertex::computeCentroid( void )
   {
      // TODO Compute the average position of all neighbors of this vertex, and
      // TODO store it in Vertex::centroid.  This value will be used for resampling.
      centroid = Vector3D(0, 0, 0);
      int degree = 0;

      HalfedgeIter h = halfedge();
      do {
         centroid = centroid + h->twin()->vertex()->position;
         degree++;
         h = h->twin()->next();
      } while(h != halfedge());
      
      centroid = centroid / degree;
   }

   Vector3D Vertex::normal( void ) const
   // TODO Returns an approximate unit normal at this vertex, computed by
   // TODO taking the area-weighted average of the normals of neighboring
   // TODO triangles, then normalizing.
   {
      // TODO Compute and return the area-weighted unit normal.
      HalfedgeCIter h = halfedge();
      Vector3D normal(0, 0, 0);

      Vector3D p0 = h->vertex()->position;
      Vector3D p1 = h->next()->vertex()->position;
      Vector3D p2;

      do {
         h = h->twin()->next();
         p2 = h->next()->vertex()->position;
         normal = normal + cross(p2-p1, p1-p0);
         p1 = p2;
      } while (h != halfedge());

      return normal.unit();
   }

   void MeshResampler::resample( HalfedgeMesh& mesh )
   {
      // TODO Compute the mean edge length.
      double mean_len = 0.0;
      EdgeIter e = mesh.edgesBegin();
      while (e != mesh.edgesEnd()) {
         // Get the next edge NOW!
         EdgeIter nextEdge = e;
         nextEdge++;

         // Sum the length
         mean_len += e->length();

         e = nextEdge;
      }
      mean_len /= mesh.nEdges();

      double min_len = mean_len * (4.f/5);
      double max_len = mean_len * (4.f/3);
      
      // TODO Repeat the four main steps for 5 or 6 iterations
      for (int main_step = 0; main_step < 6; main_step++) {
         // cout<<"main_step: "<< main_step <<endl;
         
         vector<EdgeIter> split_edge_vector;
         vector<EdgeIter> Vector;
         map<EdgeIter, int> Map;
         int id = 0;
         set<int> collapse_set;

         e = mesh.edgesBegin();
         while (e != mesh.edgesEnd()) {
            // Get the next edge NOW!
            EdgeIter nextEdge = e;
            nextEdge++;

            // If the edge is boundary
            if (!e->isBoundary() &&!e->halfedge()->isBoundary() && \
                !e->halfedge()->twin()->isBoundary() && e->length() > max_len) {
               split_edge_vector.push_back(e);
            }
            e = nextEdge;
         }
         int i;
         
         // TODO Split edges much longer than the target length (being careful about how the loop is written!)
         for (i = 0; i < split_edge_vector.size(); i++) {
               mesh.splitEdge(split_edge_vector[i]);
         }
         
         // TODO Collapse edges much shorter than the target length.  Here we need to be EXTRA careful about
         // TODO advancing the loop, because many edges may have been destroyed by a collapse (which ones?)
           
         e = mesh.edgesBegin();
         while (e != mesh.edgesEnd()) {
            // Get the next edge NOW!
            EdgeIter nextEdge = e;
            nextEdge++;

            Map.insert(pair<EdgeIter, int>(e,id));
            Vector.push_back(e);
            if (!e->isBoundary() && !e->halfedge()->isBoundary() &&
               !e->halfedge()->twin()->isBoundary()) {
               if (e->length() < min_len)
                  collapse_set.insert(id);
               }
            e = nextEdge;
            id = id + 1;
         }

         while (!collapse_set.empty()) {
            EdgeIter e = Vector[*collapse_set.begin()];
            collapse_set.erase(collapse_set.begin());
               
            if (e->length() < min_len) {
               collapse_set.erase(Map[e->halfedge()->twin()->next()->next()->edge()]);
               collapse_set.erase(Map[e->halfedge()->next()->edge()]);
               
               mesh.collapseEdge(e);
            }
         }

         // TODO Now flip each edge if it improves vertex degree         
         e = mesh.edgesBegin();
         while (e != mesh.edgesEnd()) {
            // Get the next edge NOW!
            EdgeIter nextEdge = e;
            nextEdge++;
            
            // If the edge is boundary
            if (!e->isBoundary() && !e->halfedge()->isBoundary() && \
                !e->halfedge()->twin()->isBoundary()) {

               // Get vertexes
               VertexIter a = e->halfedge()->next()->next()->vertex();
               VertexIter b = e->halfedge()->vertex();
               VertexIter c = e->halfedge()->twin()->vertex();
               VertexIter d = e->halfedge()->twin()->next()->next()->vertex();

               // Compute degree
               int degree_before, degree_after;
               degree_after   = fabs((int)a->degree()-5) + fabs((int)b->degree()-7);
               degree_after  += fabs((int)c->degree()-7) + fabs((int)d->degree()-5);
               degree_before  = fabs((int)a->degree()-6) + fabs((int)b->degree()-6);
               degree_before += fabs((int)c->degree()-6) + fabs((int)d->degree()-6);
                                      
               // Compare degree
               if (degree_before > degree_after)
                  mesh.flipEdge(e);
            }
            e = nextEdge;
         }

         // TODO Finally, apply some tangential smoothing to the vertex positions         
         double w = 0.2;
         for (int times = 0; times < 6; times++) {
            // Move each vertex in the tangent direction toward its centroid
            VertexIter v = mesh.verticesBegin();
            while (v != mesh.verticesEnd()) {
               // Get the next edge NOW!
               VertexIter nextVertex = v;
               nextVertex++;

               // Compute centroid
               v->computeCentroid();
               Vector3D n = v->normal();
               Vector3D d = v->centroid - v->position;
               v->newPosition = v->position + w * (d - dot(n,d) * n);

               v = nextVertex;
            }

            // Update the position of each vertex
            v = mesh.verticesBegin();
            while (v != mesh.verticesEnd()) {
               // Get the next edge NOW!
               VertexIter nextVertex = v;
               nextVertex++;

               // Update position
               v->position = v->newPosition;

               v = nextVertex;
            }
         }
      }
   }
}
