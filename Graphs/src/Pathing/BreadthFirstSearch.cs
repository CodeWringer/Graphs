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

            return Utility.ConstructPath(nodeCurrent, pntStart, pntGoal, cameFrom);
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
        /// <param name="paths">A grid of 'came from' locations. </param>
        /// <param name="pntStart">A starting location. </param>
        /// <returns></returns>
        public static IEnumerable<Point> GetPath(Point?[,] paths, Point pntGoal)
        {
            return Utility.ConstructPath(paths, pntGoal);
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

            frontier.Enqueue(oGrid.GetAt(pntStart.X, pntStart.Y));
            cameFrom[pntStart.X, pntStart.Y] = null;
            SquareCell nodeCurrent = null;

            // Traverse map. 
            while (frontier.Count() != 0)
            {
                nodeCurrent = frontier.Dequeue();

                if (nodeCurrent == oGrid.GetAt(pntGoal.X, pntGoal.Y)) // Reached goal destination. 
                    break;

                IEnumerable<SquareCell> neighbors = oGrid.GetNeighbors(nodeCurrent.Location, true);

                for (int next = 0; next < neighbors.Count(); next++)
                {
                    SquareCell nodeNext = neighbors.ElementAt(next);

                    if (oGrid.GetAt(nodeNext.X, nodeNext.Y).impassable) // Looking at impassable tile. 
                        continue;

                    if (cameFrom[nodeNext.X, nodeNext.Y] == null)
                    {
                        frontier.Enqueue(nodeNext);
                        cameFrom[nodeNext.X, nodeNext.Y] = nodeCurrent;
                    }
                }
            }

            return Utility.ConstructPath(nodeCurrent, pntStart, pntGoal, cameFrom);
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

            frontier.Enqueue(oGrid.GetAt(pntStart.X, pntStart.Y));
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

                    if (pntNext == oGrid.GetAt(pntStart.X, pntStart.Y)) // Ignore starting tile while looking for neighbors. 
                        continue;

                    if (oGrid.GetAt(pntNext.X, pntNext.Y).impassable) // Looking at impassable tile. 
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
        /// <param name="paths">A grid of 'came from' locations. </param>
        /// <param name="pntStart">A starting location. </param>
        /// <returns></returns>
        public static IEnumerable<SquareCell> GetPath(SquareCell[,] paths, Point pntGoal)
        {
            return Utility.ConstructPath(paths, pntGoal);
        }

        #endregion SquareGrid

        #region HexGrid



        #endregion HexGrid

        #region PolygonGrid



        #endregion PolygonGrid

        #endregion Methods
    }
}
