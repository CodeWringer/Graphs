using System;
using System.Collections.Generic;

namespace Graph
{
    /// <summary>
    /// Represents a rectangular grid in two dimensional space. 
    /// Holds vertices with additional information such as weight, edges and neighbors. 
    /// </summary>
    public class Graph : IGraph2<Vertex>
    {
        /*****************************************************************/
        // Declarations
        /*****************************************************************/
        #region Declarations

        /// <summary>
        /// A one dimensional array, representing the graph. 
        /// </summary>
        private List<Vertex> graph;

        #endregion Declarations
        /*****************************************************************/
        // Constructors
        /*****************************************************************/
        #region Constructors

        public Graph()
        {
            this.graph = new List<Vertex>();
        }

        #endregion Constructors
        /*****************************************************************/
        // Methods
        /*****************************************************************/
        #region Methods

        /// <summary>
        /// Adds an edge between the given vertices, if possible and returns true, if successful. 
        /// </summary>
        /// <param name="vertexA"></param>
        /// <param name="vertexB"></param>
        /// <returns></returns>
        public bool AddEdge(Vertex vertexA, Vertex vertexB)
        {
            bool successA = vertexA.AddEdge(vertexB);
            bool successB = vertexB.AddNeighbor(vertexA);
            bool successC = vertexA.AddNeighbor(vertexB);

            return successA && successB && successC;
        }

        /// <summary>
        /// Adds the given vertex to the graph, if possible. 
        /// </summary>
        /// <param name="vertex"></param>
        /// <returns></returns>
        public bool AddVertex(Vertex vertex)
        {
            if (!this.graph.Contains(vertex))
            {
                this.graph.Add(vertex);
                return true;
            }
            return false;
        }

        public IEnumerable<Vertex> GetNeighbors(Vertex vertex)
        {
            return vertex.GetNeighbors();
        }

        public bool IsAdjacent(Vertex vertexA, Vertex vertexB)
        {
            if (vertexA.HasNeighbor(vertexB))
            {
                return true;
            }
            return false;
        }

        public bool RemoveEdge(Vertex vertexA, Vertex vertexB)
        {
            bool successA = vertexA.RemoveEdge(vertexB);
            bool successB = vertexA.AddNeighbor(vertexB);
            bool successC = vertexB.RemoveNeighbor(vertexA);

            return successA && successB && successC;
        }

        /// <summary>
        /// Removes the given vertex. 
        /// </summary>
        /// <param name="vertex"></param>
        /// <returns></returns>
        public bool RemoveVertex(Vertex vertex)
        {
            IEnumerable<Vertex> neighbors = this.GetNeighbors(vertex);

            // Remove all connections to given vertex. 
            foreach (Vertex neighbor in neighbors)
            {
                neighbor.RemoveEdge(vertex);
            }
            // Remove given vertex. 
            this.graph[this.IndexOf(vertex)] = null;

            throw new NotImplementedException();
        }

        /// <summary>
        /// Returns the vertex at the given coordinates. 
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        public Vertex GetVertex(int x, int y)
        {
            foreach (Vertex v in this.graph)
            {
                if (v.X == x && v.Y == y)
                {
                    return v;
                }
            }
            return null;
        }

        /// <summary>
        /// Returns the index of the given vertex or -1 if it's not contained. 
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public int IndexOf(Vertex v)
        {
            for (int i = 0; i < this.graph.Count; i++)
            {
                if (this.graph[i] == v)
                {
                    return i;
                }
            }
            return -1;
        }

        #endregion Methods
    }

    /// <summary>
    /// Represents a vertex of the grid. 
    /// </summary>
    public class Vertex
    {
        /*****************************************************************/
        // Declarations
        /*****************************************************************/
        #region Declarations

        /// <summary>
        /// X-coordinate of this vertex on the grid. 
        /// </summary>
        public int X { get; private set; }

        /// <summary>
        /// Y-coordinate of this vertex on the grid. 
        /// </summary>
        public int Y { get; private set; }

        /// <summary>
        /// Weight of this vertex, for use in pathfinding. 
        /// </summary>
        public int weight;

        /// <summary>
        /// A list of neighboring (adjacent) vertices. 
        /// </summary>
        private List<Vertex> lNeighbor;

        /// <summary>
        /// A list of edges originating from this vertex. 
        /// </summary>
        private List<Edge> lEdge;

        /// <summary>
        /// If true, this vertex can be used for pathfinding. 
        /// </summary>
        public bool walkable;

        #endregion Declarations
        /*****************************************************************/
        // Constructors
        /*****************************************************************/
        #region Constructors

        public Vertex(int x, int y)
        {
            this.X = x;
            this.Y = y;
            this.lNeighbor = new List<Vertex>();
            this.lEdge = new List<Edge>();
            this.walkable = true;
        }

        public Vertex(int x, int y, int weight)
            : this(x, y)
        {
            this.weight = weight;
        }

        #endregion Constructors
        /*****************************************************************/
        // Methods
        /*****************************************************************/
        #region Methods

        /// <summary>
        /// Adds a neighbor, if possible and returns true, if successful. 
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public bool AddNeighbor(Vertex v)
        {
            if (!this.lNeighbor.Contains(v))
            {
                this.lNeighbor.Add(v);
                return true;
            }
            return false;
        }

        /// <summary>
        /// Adds a neighbor, if possible and returns true, if successful. 
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public bool RemoveNeighbor(Vertex v)
        {
            return this.lNeighbor.Remove(v);
        }

        /// <summary>
        /// Returns true, if this vertex has the given vertex as its neighbor. 
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public bool HasNeighbor(Vertex v)
        {
            foreach (Vertex neighbor in this.lNeighbor)
            {
                if (neighbor == v)
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public IEnumerable<Vertex> GetNeighbors()
        {
            return this.lNeighbor;
        }

        /// <summary>
        /// Returns the edge that connects this vertex with the given node. 
        /// Returns null, if there is no connection. 
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public Edge GetEdge(Vertex v)
        {
            foreach (Edge edge in this.lEdge)
            {
                if (edge.vTarget == v)
                {
                    return edge;
                }
            }
            return null;
        }

        /// <summary>
        /// Adds an edge between this and the given vertex, if possible and returns true, if successful. 
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public bool AddEdge(Vertex v)
        {
            if (this.GetEdge(v) == null)
            {
                this.lEdge.Add(new Edge(this, v));
                return true;
            }
            return false;
        }

        /// <summary>
        /// Removes an edge between this and the given vertex, if possible and returns true, if successful. 
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public bool RemoveEdge(Vertex v)
        {
            Edge edge = this.GetEdge(v);

            if (edge == null)
            {
                return false;
            }
            this.lEdge.Remove(edge);
            return true;
        }

        #endregion Methods
    }

    /// <summary>
    /// Represents an edge between two vertices. 
    /// </summary>
    public class Edge
    {
        /*****************************************************************/
        // Declarations
        /*****************************************************************/
        #region Declarations

        /// <summary>
        /// A vertex this edge originates from. 
        /// </summary>
        public Vertex vOrigin { get; private set; }

        /// <summary>
        /// A vertex this edge terminates at. 
        /// </summary>
        public Vertex vTarget { get; private set; }

        /// <summary>
        /// Weight of this edge, for use in pathfinding. 
        /// </summary>
        public int weight;

        /// <summary>
        /// If true, this edge can be used for pathfinding. 
        /// </summary>
        public bool walkable;

        #endregion Declarations
        /*****************************************************************/
        // Constructors
        /*****************************************************************/
        #region Constructors

        public Edge(Vertex vOrigin, Vertex vTarget)
        {
            this.vOrigin = vOrigin;
            this.vTarget = vTarget;
            this.walkable = true;
        }

        public Edge(Vertex vOrigin, Vertex vTarget, int weight)
            : this(vOrigin, vTarget)
        {
            this.weight = weight;
        }

        #endregion Constructors
        /*****************************************************************/
        // Methods
        /*****************************************************************/
        #region Methods

        #endregion Methods
    }
}
