using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;

using Graph.Grid;

namespace Graph.Pathing
{
    /// <summary>
    /// An internal utility class for various graph operations. 
    /// </summary>
    internal abstract class Utility
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

        /// <summary>
        /// Returns the coordinates of all neighboring tiles. 
        /// </summary>
        /// <param name="pnt">A point for which all neighbors are to be returned. </param>
        /// <param name="grid">A gid to search on. </param>
        /// <param name="allowDiagonal">If true, also returns diagonal neighbors. </param>
        /// <returns></returns>
        internal static IEnumerable<Point> GetNeighbors(Point pnt, int[,] grid, bool allowDiagonal)
        {
            List<Point> neighbors = new List<Point>();

            int xAt = -1;
            int yAt = -1;

            // Get axis aligned neighbors. 
            for (int i = 0; i < SquareGrid.Directions.Length; i++)
            {
                xAt = pnt.X + SquareGrid.Directions[i].X;
                yAt = pnt.Y + SquareGrid.Directions[i].Y;

                if (!Utility.IsOutOfBounds(xAt, yAt, grid))
                {
                    neighbors.Add(new Point(xAt, yAt));
                }
            }

            if (allowDiagonal)
            {
                // Get diagonal neighbors. 
                for (int i = 0; i < SquareGrid.DirectionsDiag.Length; i++)
                {
                    xAt = pnt.X + SquareGrid.DirectionsDiag[i].X;
                    yAt = pnt.Y + SquareGrid.DirectionsDiag[i].Y;

                    if (!Utility.IsOutOfBounds(xAt, yAt, grid))
                    {
                        neighbors.Add(new Point(xAt, yAt));
                    }
                }
            }

            return neighbors;
        }

        /// <summary>
        /// Returns true, if the given coordinates represent a diagonal direction. 
        /// </summary>
        /// <param name="x">A x-coordinate in the range of -1 to +1. </param>
        /// <param name="y">A y-coordinate in the range of -1 to +1. </param>
        /// <returns></returns>
        internal static bool IsDiagonal(int x, int y)
        {
            if (x == 0 || y == 0)
                return false;
            else
                return true;
        }

        /// <summary>
        /// Returns true, if the given coordinates are out of bounds. 
        /// </summary>
        /// <param name="x">A x-coordinate. </param>
        /// <param name="y">A y-coordinate. </param>
        /// <returns></returns>
        internal static bool IsOutOfBounds(int x, int y, int[,] grid)
        {
            if (x < 0 || y < 0 || x >= grid.GetLength(0) || y >= grid.GetLength(1))
            {
                return true;
            }

            return false;
        }

        /// <summary>
        /// Returns the cost difference between the given points on the given grid of traversal costs. 
        /// </summary>
        /// <param name="gridCost">A grid of traversal costs. </param>
        /// <param name="pntA">A point. </param>
        /// <param name="pntB">A neighbor point. </param>
        /// <param name="costDiagonal">A cost factor for diagonal tiles. A value of less than 
        /// or equal to 0 won't be used. Can be used to weigh diagonal tiles higher. </param>
        /// <returns></returns>
        internal static float GetCost(int[,] gridCost, Point pntA, Point pntB, float costDiagonal = 1.0F)
        {
            float cost = Math.Max(gridCost[pntB.X, pntB.Y] - gridCost[pntA.X, pntA.Y], 1);

            return costDiagonal > 0 ? (cost * costDiagonal) : cost;
        }

        /// <summary>
        /// Returns the cost difference between the given square cells. 
        /// </summary>
        /// <param name="cellA">A cell. </param>
        /// <param name="cellB">A neighboring cell. </param>
        /// <param name="costDiagonal">A cost factor for diagonal tiles. A value of less than 
        /// or equal to 0 won't be used. Can be used to weigh diagonal tiles higher. </param>
        /// <returns></returns>
        internal static float GetCost(SquareCell cellA, SquareCell cellB, float costDiagonal = 1.0F)
        {
            float cost = Math.Max(cellB.cost - cellA.cost, 1.0F);

            return costDiagonal > 0 ? (cost * costDiagonal) : cost;
        }

        /// <summary>
        /// Returns the Manhattan distance. 
        /// </summary>
        /// <param name="goal">A point representing the goal position. </param>
        /// <param name="point">A point to get the heuristic for. </param>
        /// <param name="D">The minimum cost between any adjacent nodes. </param>
        /// <returns></returns>
        internal static float GetHeuristicManhattan(Point goal, Point point, float D)
        {
            float dx = Math.Abs(point.X - goal.X);
            float dy = Math.Abs(point.Y - goal.Y);
            return D * (dx + dy);
        }

        /// <summary>
        /// Returns the diagonal distance. 
        /// </summary>
        /// <param name="goal">A point representing the goal position. </param>
        /// <param name="point">A point to get the heuristic for. </param>
        /// <param name="D">The minimum cost between any adjacent nodes. </param>
        /// <param name="D2">The cost of moving diagonally. </param>
        /// <returns></returns>
        internal static float GetHeuristicDiagonal(Point goal, Point point, float D, float D2)
        {
            float dx = Math.Abs(point.X - goal.X);
            float dy = Math.Abs(point.Y - goal.Y);
            return D * (dx + dy) + (D2 - 2 * D) * Math.Min(dx, dy);
        }

        /// <summary>
        /// Returns the Euclidean distance. 
        /// </summary>
        /// <param name="goal"></param>
        /// <param name="point"></param>
        /// <param name="D"></param>
        /// <returns></returns>
        internal static float GetHeuristicEuclidean(Point goal, Point point, float D)
        {
            float dx = Math.Abs(point.X - goal.X);
            float dy = Math.Abs(point.Y - goal.Y);
            return D * (float)Math.Sqrt(dx * dx + dy * dy);
        }
        
        /// <summary>
        /// Returns the lowest cost of all the neighboring nodes of the given node. 
        /// </summary>
        /// <param name="node"></param>
        /// <param name="oGrid"></param>
        /// <param name="costDiagonal"></param>
        /// <returns></returns>
        internal static float GetLowestCost(SquareCell node, SquareGrid oGrid, float costDiagonal)
        {
            IEnumerable<SquareCell> neighbors = null;
            float costLowest = float.MaxValue;

            if (costDiagonal <= 0)
                neighbors = oGrid.GetNeighbors(node.Location, false);
            else
                neighbors = oGrid.GetNeighbors(node.Location, true);

            foreach (SquareCell nodeNeighbor in neighbors)
            {
                if (nodeNeighbor.cost < costLowest)
                    costLowest = nodeNeighbor.cost;
            }
            return costLowest;
        }

        /// <summary>
        /// Returns the lowest cost of all the neighboring nodes of the given node. 
        /// </summary>
        /// <param name="node"></param>
        /// <param name="costDiagonal"></param>
        /// <returns></returns>
        internal static float GetLowestCost(Point node, int[,] grid, int[,] gridCost, float costDiagonal)
        {
            IEnumerable<Point> neighbors = null;
            float costLowest = float.MaxValue;

            if (costDiagonal <= 0)
                neighbors = Utility.GetNeighbors(node, grid, false);
            else
                neighbors = Utility.GetNeighbors(node, grid, true);

            foreach (Point nodeNeighbor in neighbors)
            {
                float costNeighbor = gridCost[nodeNeighbor.X, nodeNeighbor.Y];
                if (costNeighbor < costLowest)
                    costLowest = costNeighbor;
            }
            return costLowest;
        }

        /// <summary>
        /// Returns a path constructed from the given parameters. 
        /// </summary>
        /// <param name="nodeCurrent"></param>
        /// <param name="pntStart"></param>
        /// <param name="pntGoal"></param>
        /// <param name="cameFrom"></param>
        /// <returns></returns>
        internal static IEnumerable<Point> ConstructPath(Point nodeCurrent, Point pntStart, Point pntGoal, Point?[,] cameFrom)
        {
            if (cameFrom[pntGoal.X, pntGoal.Y] == null) // Could not find path. 
                return null;

            List<Point> lPath = new List<Point>();
            nodeCurrent = pntGoal;
            lPath.Add(nodeCurrent);
            while (nodeCurrent != pntStart)
            {
                nodeCurrent = cameFrom[nodeCurrent.X, nodeCurrent.Y].Value;
                lPath.Add(nodeCurrent);
            }
            lPath.Add(pntStart);
            lPath.Reverse();

            return lPath;
        }

        /// <summary>
        /// Returns a path constructed from the given parameters. 
        /// </summary>
        /// <param name="node"></param>
        /// <param name="pntStart"></param>
        /// <param name="pntGoal"></param>
        /// <param name="oGrid"></param>
        /// <param name="cameFrom"></param>
        /// <returns></returns>
        internal static IEnumerable<SquareCell> ConstructPath(SquareCell node, Point pntStart, Point pntGoal, SquareCell[,] cameFrom)
        {
            if (cameFrom[pntGoal.X, pntGoal.Y] == null) // Could not find path. 
                return null;

            List<SquareCell> lPath = new List<SquareCell>();
            node = cameFrom[pntGoal.X, pntGoal.Y];
            lPath.Add(node);
            while (node != cameFrom[pntStart.X, pntStart.Y])
            {
                node = cameFrom[node.X, node.Y];
                lPath.Add(node);
            }
            lPath.Add(cameFrom[pntStart.X, pntStart.Y]);
            lPath.Reverse();

            return lPath;
        }

        /// <summary>
        /// Returns a path constructed from the given grid, at the given end location. 
        /// The path leads to the starting location that can be found via the grid traversal. 
        /// </summary>
        /// <param name="paths">A grid of 'came from' locations. </param>
        /// <param name="pntStart">A starting location. </param>
        /// <returns></returns>
        public static IEnumerable<SquareCell> ConstructPath(SquareCell[,] paths, Point pntGoal)
        {
            List<SquareCell> lPath = new List<SquareCell>();

            // Construct path. 
            lPath.Add(new SquareCell(pntGoal.X, pntGoal.Y));
            SquareCell current = paths[pntGoal.X, pntGoal.Y];
            lPath.Add(current);

            if (current == null)
                return lPath;

            while (paths[current.X, current.Y] != null)
            {
                current = paths[current.X, current.Y];
                lPath.Add(current);
            }
            lPath.Reverse();

            return lPath;
        }

        /// <summary>
        /// Returns a path constructed from the given grid, at the given end location. 
        /// The path leads to the starting location that can be found via the grid traversal. 
        /// </summary>
        /// <param name="paths">A grid of 'came from' locations. </param>
        /// <param name="pntStart">A starting location. </param>
        /// <returns></returns>
        public static IEnumerable<Point> ConstructPath(Point?[,] paths, Point pntGoal)
        {
            List<Point> lPath = new List<Point>();

            // Construct path. 
            lPath.Add(pntGoal);
            Point current = paths[pntGoal.X, pntGoal.Y].Value;
            lPath.Add(current);

            if (current == null)
                return lPath;

            while (paths[current.X, current.Y] != null)
            {
                current = paths[current.X, current.Y].Value;
                lPath.Add(current);
            }
            lPath.Reverse();

            return lPath;
        }

        #endregion Methods
    }
}
