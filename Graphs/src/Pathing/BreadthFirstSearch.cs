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

        #region GenericGrid

        /// <summary>
        /// Finds a path on the given grid and returns the path, beginning with the given start node. 
        /// </summary>
        /// <remarks>
        /// Does an early exit. 
        /// </remarks>
        /// <param name="pntStart">A node to begin the search at. </param>
        /// <param name="pntEnd">A node to end the search at. </param>
        /// <param name="grid">The grid to search on. </param>
        /// <param name="tileImpassable">The number associated with impassable tiles. </param>
        /// <returns></returns>
        public static IEnumerable<Point> GetPath(Point pntStart, Point pntEnd, int[,] grid, int tileImpassable)
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

                if (current == pntEnd) // Reached goal destination. 
                    break;

                IEnumerable<Point> neighbors = BreadthFirstSearch.GetNeighbors(current, grid);

                for (int next = 0; next < neighbors.Count(); next++)
                {
                    Point pntNext = neighbors.ElementAt(next);

                    if (grid[pntNext.X, pntNext.Y] == tileImpassable) // Looking at impassable tile. 
                        continue;

                    if (cameFrom[pntNext.X, pntNext.Y] == null)
                    {
                        frontier.Enqueue(pntNext);
                        cameFrom[pntNext.X, pntNext.Y] = current;
                    }
                }
            }

            if (cameFrom[pntEnd.X, pntEnd.Y] == null) // Could not find path. 
                return null;

            // Construct path. 
            current = pntEnd;
            lPath.Add(current);
            while (current != pntStart)
            {
                current = cameFrom[current.X, current.Y].Value;
                lPath.Add(current);
            }
            lPath.Add(pntStart);
            lPath.Reverse();

            return lPath;
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
                
                IEnumerable<Point> neighbors = BreadthFirstSearch.GetNeighbors(current, grid);

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
        public static IEnumerable<Point> GetPath(Point?[,] paths, Point pntEnd)
        {
            List<Point> lPath = new List<Point>();

            // Construct path. 
            lPath.Add(pntEnd);
            Point current = paths[pntEnd.X, pntEnd.Y].Value;
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

        /// <summary>
        /// Returns the coordinates of all neighboring tiles. 
        /// </summary>
        /// <param name="pnt"></param>
        /// <param name="grid"></param>
        /// <returns></returns>
        private static IEnumerable<Point> GetNeighbors(Point pnt, int[,] grid)
        {
            List<Point> neighbors = new List<Point>();

            Point[] Directions = new Point[] {
                SquareGrid.East,
                SquareGrid.South,
                SquareGrid.West,
                SquareGrid.North,
                SquareGrid.SEast,
                SquareGrid.SWest,
                SquareGrid.NWest,
                SquareGrid.NEast
            };

            int xAt = -1;
            int yAt = -1;

            for (int i = 0; i < Directions.Length; i++)
            {
                xAt = pnt.X + Directions[i].X;
                yAt = pnt.Y + Directions[i].Y;

                if (!BreadthFirstSearch.IsOutOfBounds(xAt, yAt, grid))
                {
                    neighbors.Add(new Point(xAt, yAt));
                }
            }

            return neighbors;
        }

        /// <summary>
        /// Returns true, if the given coordinates are out of bounds. 
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        private static bool IsOutOfBounds(int x, int y, int[,] grid)
        {
            if (x < 0 || y < 0 || x >= grid.GetLength(0) || y >= grid.GetLength(1))
            {
                return true;
            }

            return false;
        }

        #endregion GenericGrid

        #region SquareGrid

        /// <summary>
        /// Finds a path on the given grid and returns the path, beginning with the given start node. 
        /// </summary>
        /// <param name="pntStart">A node to begin the search at. </param>
        /// <param name="pntEnd">A node to end the search at. </param>
        /// <param name="oGrid">The grid do to the search with. </param>
        /// <returns></returns>
        public static IEnumerable<SquareCell> GetPath(Point pntStart, Point pntEnd, SquareGrid oGrid)
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

                if (current == oGrid.GetAt(pntEnd.X, pntEnd.Y)) // Reached goal destination. 
                    break;

                IEnumerable<SquareCell> neighbors = oGrid.GetNeighbors(current.Location, true);

                for (int next = 0; next < neighbors.Count(); next++)
                {
                    SquareCell pntNext = neighbors.ElementAt(next);

                    if (oGrid.GetAt(pntNext.X, pntNext.Y).impassable) // Looking at impassable tile. 
                        continue;

                    if (cameFrom[pntNext.X, pntNext.Y] == null)
                    {
                        frontier.Enqueue(pntNext);
                        cameFrom[pntNext.X, pntNext.Y] = current;
                    }
                }
            }

            if (cameFrom[pntEnd.X, pntEnd.Y] == null) // Could not find path. 
                return null;

            // Construct path. 
            current = oGrid.GetAt(pntEnd.X, pntEnd.Y);
            lPath.Add(current);
            while (current != oGrid.GetAt(pntStart.X, pntStart.Y))
            {
                current = cameFrom[current.X, current.Y];
                lPath.Add(current);
            }
            lPath.Add(oGrid.GetAt(pntStart.X, pntStart.Y));
            lPath.Reverse();

            return lPath;
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
        public static IEnumerable<SquareCell> GetPath(SquareCell[,] paths, Point pntEnd)
        {
            List<SquareCell> lPath = new List<SquareCell>();

            // Construct path. 
            lPath.Add(new SquareCell(pntEnd.X, pntEnd.Y));
            SquareCell current = paths[pntEnd.X, pntEnd.Y];
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

        #endregion SquareGrid

        #region HexGrid



        #endregion HexGrid

        #region PolygonGrid



        #endregion PolygonGrid

        #endregion Methods
        /*****************************************************************/
        // Events
        /*****************************************************************/
        #region Events

        #endregion Events
    }
}
