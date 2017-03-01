using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

using Tools.Maths.Point3;
using Graph.Grid;

namespace Graph.Pathing
{
    /// <summary>
    /// Pathfinding algorithm 'BreadthFirstSearch'. 
    /// Useful for finding all source destinations for a given target destination or vice versa. 
    /// </summary>
    public abstract class BreadthFirstSearch
    {
        /*****************************************************************/
        // Declarations
        /*****************************************************************/
        #region Declarations

        #endregion Declarations
        /*****************************************************************/
        // Methods
        /*****************************************************************/
        #region Methods

        #region SimpleGrid

        /// <summary>
        /// Finds a path on the given grid and returns the path, beginning with the given start node. 
        /// </summary>
        /// <remarks>
        /// Does an early exit. 
        /// </remarks>
        /// <param name="pntStart">A node to begin the search at. </param>
        /// <param name="pntGoal">A node to end the search at. </param>
        /// <param name="grid">The grid to search on. </param>
        /// <param name="tileImpassable">The number associated with impassable tiles. </param>
        /// <returns></returns>
        public static IEnumerable<Point> GetPath(Point pntStart, Point pntGoal, int[,] grid, int tileImpassable)
        {
            Queue<Point> frontier = new Queue<Point>();
            List<Point> lPath = new List<Point>();
            Point?[,] cameFrom = new Point?[grid.GetLength(0), grid.GetLength(1)];

            frontier.Enqueue(pntStart);
            cameFrom[pntStart.X, pntStart.Y] = null;
            Point nodeCurrent = new Point();

            // Traverse map. 
            while (frontier.Count() != 0)
            {
                nodeCurrent = frontier.Dequeue();

                if (nodeCurrent == pntGoal) // Reached goal destination. 
                    break;

                IEnumerable<Point> neighbors = Utility.GetNeighbors(nodeCurrent, grid, true);

                for (int next = 0; next < neighbors.Count(); next++)
                {
                    Point pntNext = neighbors.ElementAt(next);

                    if (grid[pntNext.X, pntNext.Y] == tileImpassable) // Looking at impassable tile. 
                        continue;

                    if (cameFrom[pntNext.X, pntNext.Y] == null)
                    {
                        frontier.Enqueue(pntNext);
                        cameFrom[pntNext.X, pntNext.Y] = nodeCurrent;
                    }
                }
            }

            return Utility.ConstructPath(cameFrom, pntStart, pntGoal);
        }

        /// <summary>
        /// Returns a grid representing all steps required to get to the given starting point, from any location. 
        /// </summary>
        /// <remarks>
        /// Does not do an early exit. 
        /// </remarks>
        /// <param name="pntStart">A node to begin the search at. </param>
        /// <param name="grid">The grid to search on. </param>
        /// <param name="tileImpassable">The number associated with impassable tiles. </param>
        /// <returns></returns>
        public static Point?[,] GetPath(Point pntStart, int[,] grid, int tileImpassable)
        {
            Queue<Point> frontier = new Queue<Point>();
            List<Point> lPath = new List<Point>();
            Point?[,] cameFrom = new Point?[grid.GetLength(0), grid.GetLength(1)];

            frontier.Enqueue(pntStart);
            cameFrom[pntStart.X, pntStart.Y] = null;
            Point current;

            // Traverse map. 
            while (frontier.Count() != 0)
            {
                current = frontier.Dequeue();
                
                IEnumerable<Point> neighbors = Utility.GetNeighbors(current, grid, true);

                for (int next = 0; next < neighbors.Count(); next++)
                {
                    Point pntNext = neighbors.ElementAt(next);

                    if (pntNext == pntStart) // Ignore starting tile while looking for neighbors. 
                        continue;

                    if (grid[pntNext.X, pntNext.Y] == tileImpassable) // Looking at impassable tile. 
                        continue;

                    if (cameFrom[pntNext.X, pntNext.Y] == null)
                    {
                        frontier.Enqueue(pntNext);
                        cameFrom[pntNext.X, pntNext.Y] = current;
                    }
                }
            }
            
            return cameFrom;
        }

        /// <summary>
        /// Returns a path constructed from the given grid, at the given end location. 
        /// The path leads to the starting location that can be found via the grid traversal. 
        /// </summary>
        /// <param name="cameFrom">A grid of 'came from' locations. </param>
        /// <param name="pntStart">A starting location. </param>
        /// <returns></returns>
        public static IEnumerable<Point> GetPath(Point?[,] cameFrom, Point pntGoal)
        {
            return Utility.ConstructPath(cameFrom, pntGoal);
        }

        #endregion SimpleGrid

        #region SquareGrid

        /// <summary>
        /// Finds a path on the given grid and returns the path, beginning with the given start node. 
        /// </summary>
        /// <param name="pntStart">A node to begin the search at. </param>
        /// <param name="pntGoal">A node to end the search at. </param>
        /// <param name="oGrid">The grid do to the search with. </param>
        /// <returns></returns>
        public static IEnumerable<SquareCell> GetPath(Point pntStart, Point pntGoal, SquareGrid oGrid)
        {
            Queue<SquareCell> frontier = new Queue<SquareCell>();
            List<SquareCell> lPath = new List<SquareCell>();
            SquareCell[,] cameFrom = new SquareCell[oGrid.Width, oGrid.Height];

            frontier.Enqueue(oGrid.GetNode(pntStart.X, pntStart.Y));
            cameFrom[pntStart.X, pntStart.Y] = null;
            SquareCell nodeCurrent = null;

            // Traverse map. 
            while (frontier.Count() != 0)
            {
                nodeCurrent = frontier.Dequeue();

                if (nodeCurrent == oGrid.GetNode(pntGoal.X, pntGoal.Y)) // Reached goal destination. 
                    break;

                IEnumerable<SquareCell> neighbors = oGrid.GetNeighbors(nodeCurrent.Location, true);

                for (int next = 0; next < neighbors.Count(); next++)
                {
                    SquareCell nodeNext = neighbors.ElementAt(next);

                    if (nodeNext.impassable) // Looking at impassable tile. 
                        continue;

                    if (cameFrom[nodeNext.X, nodeNext.Y] == null)
                    {
                        frontier.Enqueue(nodeNext);
                        cameFrom[nodeNext.X, nodeNext.Y] = nodeCurrent;
                    }
                }
            }

            return Utility.ConstructPath(cameFrom, pntStart, pntGoal);
        }

        /// <summary>
        /// Returns a grid representing all steps required to get to the given starting point, from any location. 
        /// </summary>
        /// <param name="pntStart">A node to begin the search at. </param>
        /// <param name="oGrid">The grid do to the search with. </param>
        /// <returns></returns>
        public static SquareCell[,] GetPath(Point pntStart, SquareGrid oGrid)
        {
            Queue<SquareCell> frontier = new Queue<SquareCell>();
            List<SquareCell> lPath = new List<SquareCell>();
            SquareCell[,] cameFrom = new SquareCell[oGrid.Width, oGrid.Height];

            frontier.Enqueue(oGrid.GetNode(pntStart.X, pntStart.Y));
            cameFrom[pntStart.X, pntStart.Y] = null;
            SquareCell current;

            // Traverse map. 
            while (frontier.Count() != 0)
            {
                current = frontier.Dequeue();

                IEnumerable<SquareCell> neighbors = oGrid.GetNeighbors(current.Location, true);

                for (int next = 0; next < neighbors.Count(); next++)
                {
                    SquareCell pntNext = neighbors.ElementAt(next);

                    if (pntNext == oGrid.GetNode(pntStart.X, pntStart.Y)) // Ignore starting tile while looking for neighbors. 
                        continue;

                    if (pntNext.impassable) // Looking at impassable tile. 
                        continue;

                    if (cameFrom[pntNext.X, pntNext.Y] == null)
                    {
                        frontier.Enqueue(pntNext);
                        cameFrom[pntNext.X, pntNext.Y] = current;
                    }
                }
            }

            return cameFrom;
        }

        /// <summary>
        /// Returns a path constructed from the given grid, at the given end location. 
        /// The path leads to the starting location that can be found via the grid traversal. 
        /// </summary>
        /// <param name="cameFrom">A grid of 'came from' locations. </param>
        /// <param name="pntStart">A starting location. </param>
        /// <returns></returns>
        public static IEnumerable<SquareCell> GetPath(SquareCell[,] cameFrom, Point pntGoal)
        {
            return Utility.ConstructPath(cameFrom, pntGoal);
        }

        #endregion SquareGrid

        #region HexGrid

        /// <summary>
        /// Finds a path on the given grid and returns the path, beginning with the given start node. 
        /// </summary>
        /// <param name="pntStart">A node to begin the search at. </param>
        /// <param name="pntGoal">A node to end the search at. </param>
        /// <param name="oGrid">The grid do to the search with. </param>
        /// <returns></returns>
        public static IEnumerable<HexagonCell> GetPath(csPoint3 pntStart, csPoint3 pntGoal, HexagonGrid oGrid)
        {
            Queue<HexagonCell> frontier = new Queue<HexagonCell>();
            List<HexagonCell> lPath = new List<HexagonCell>();
            HexagonCell[,] cameFrom = new HexagonCell[oGrid.Width, oGrid.Height];

            frontier.Enqueue(oGrid.GetNode(pntStart));
            cameFrom[pntStart.X, pntStart.Y] = null;
            HexagonCell nodeCurrent = null;

            // Traverse map. 
            while (frontier.Count() != 0)
            {
                nodeCurrent = frontier.Dequeue();

                if (nodeCurrent == oGrid.GetNode(pntGoal)) // Reached goal destination. 
                    break;

                IEnumerable<HexagonCell> neighbors = oGrid.GetNeighbors(nodeCurrent.Location);

                for (int next = 0; next < neighbors.Count(); next++)
                {
                    HexagonCell nodeNext = neighbors.ElementAt(next);

                    if (nodeNext.impassable) // Looking at impassable tile. 
                        continue;

                    if (cameFrom[nodeNext.X, nodeNext.Y] == null)
                    {
                        frontier.Enqueue(nodeNext);
                        cameFrom[nodeNext.X, nodeNext.Y] = nodeCurrent;
                    }
                }
            }

            return Utility.ConstructPath(cameFrom, pntStart, pntGoal);
        }

        #endregion HexGrid

        #region PolygonGrid



        #endregion PolygonGrid

        #endregion Methods
    }
}
