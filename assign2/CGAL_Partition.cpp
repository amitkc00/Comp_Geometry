#include <math.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>


//#include <gl/glut.h>  // for the OpenGL GLUT library

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

// CGAL header files

#include <CGAL/basic.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Partition_is_valid_traits_2.h>
#include <CGAL/polygon_function_objects.h>
#include <CGAL/partition_2.h>
#include <cassert>
#include <list>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Partition_traits_2<K>                         Traits;
typedef CGAL::Is_convex_2<Traits>                           Is_convex_2;
typedef Traits::Point_2                                     Point_2;
typedef Traits::Polygon_2                                   Polygon_2;
typedef std::list<Polygon_2>                                Polygon_list;
typedef CGAL::Partition_is_valid_traits_2<Traits, Is_convex_2>
                                                            Validity_traits;

typedef Polygon_2::Vertex_const_iterator VertexIterator;
typedef Polygon_2::Edge_const_iterator EdgeIterator;

using std::cout; 
using std::cin;
using std::endl;

// 2D CGAL polygon partitioning program by Prof. Daniels for 91.504, 9 Feb.
// with some elements borrowed from Shu Ye's 3D surface code (for OpenGL menu support),
// some code borrowed from OpenGL programming guide,
// and code cut-and-pasted from CGAL examples on 2D polygon partitioning.

#define MAX_N 100 // maximum number of polygon vertices
int N = 0; // number of polygon vertices
#define D 2 // number of dimensions

// Array vertices is used to display OpenGL polygon and to initialize global CGAL polygon p.
// Initialize vertices from file "poly.dat".
//   File format: 1st line contains integer number of polygon vertices, which will be stored in N
//				  Each subsequent line contains D floating point coordinates of a single vertex
//                delimited by white space.

// Stylistically I typically don't like to use global variables, but I'm making an
// exception here for convenience with OpenGL's menu event handling.

bool polyInitialized  = false;
GLfloat vertices[MAX_N][D]; // set up by call to inputPolygonFile()
							// triggered by user's selection of 
                            //"Read Polygon from File" menu choice
static Polygon_2 p;  // CGAL 2D polygon

int typeMode = -1;	 // 0 = input polygon's vertices (read from file poly.dat)
					 // 1 = draw original polygon's boundary
					 // 2 = y-monotone partition (de Berg et al. O(NlogN))
					 // 3 = optimal convex decomposition (Greene dynamic programming O(N^4))
					 // 4 = approximate convex partition  
					 //    (triangulation, then Hertel/Mehlhorn O(NlogN))
					 // 5 = greene approximate convex partition 
					 //    (y-monotone partition, sweep-line approach) O(NlogN)
					 // > 5 = exit

// window size for OpenGL
// Make sure input vertices stay within this range.
int ww = 200;
int wh = 200;

void createCGAL_Polygon()
{
	if (polyInitialized == false)
	{
		cout << "Error! Polygon has not yet been read from file poly.dat." << endl;
		return;
	}

	// Adds 2D points from global "vertices" array to global polygon p.

	printf("\nSetting up 2D Polygon Vertices:\n");

	for (int i = 0; i < N; i++)
	{
	  p.push_back(Point_2(vertices[i][0], vertices[i][1]));
	}

	CGAL::set_pretty_mode(cout);
    cout << "created the polygon p:" << endl;
    cout << p << endl;
    cout << endl;
}

void inputPolygonFile()
{
 // Read polygon from file poly.dat and initialize both
 // vertices array and CGAL polygon.

//   File format: 1st line contains integer number of polygon vertices, which will be stored in N
//				  Each subsequent line contains D floating point coordinates of a single vertex
//                delimited by white space.

 if (polyInitialized == true)
 {
	 cout << "Warning! Polygon has already been initialized, so we cannot reinitialize." << endl;
	 return;
 }

 // Open input file poly.dat
 std::ifstream inFile("poly.dat");
 if (!inFile.is_open())
 {
	 cout << "Error attempting to open input file poly.dat!" << endl;
	 return;
 }
 if (inFile.eof())
	 {
	 cout << "Error attempting to read number of polygon vertices from input file poly.dat!" << endl;
	 inFile.close();
	 return;
 }
 inFile >> N;
 if (N > MAX_N)
 {
	 cout << "Error!  Number of polygon vertices exceeds maximum = " << MAX_N << endl;
	 inFile.close();
	 return;
 }
 for (int i = 0; i < N; i++)
 {
	 for (int j = 0; j < D; j++)
	 {
	   if (inFile.eof())
	   {
	    cout << "Error! Premature end of input file poly.dat." << endl;
		inFile.close();
		return;
	   } // end if
	   inFile >> vertices[i][j];
	   if (j == 0)
	   {
		   if (vertices[i][j] > ww || vertices[i][j] <0 )
		   {
			   cout << "Error! X coordinate of point in input file poly.dat is out of window range." << endl;
			   inFile.close();
			   return;
		   }
	   }
	   if (j == 1)
	   {
		   if (vertices[i][j] > wh || vertices[i][j] <0 )
		   {
			   cout << "Error! Y coordinate of point in input file poly.dat is out of window range." << endl;
			   inFile.close();
			   return;
		   }
	   }
	 } // end for j
 } // end for i

 inFile.close();
 polyInitialized = true;

 createCGAL_Polygon();  // Adds 2D points from global "vertices" array to global polygon p.
}

void displayCGAL_Polygon(const Polygon_2& CGAL_Poly)
{
	cout << endl << "Displaying CGAL polygon..." << endl;

	if (polyInitialized == false)
	{
		cout << "Error! Polygon has not yet been read from file poly.dat." << endl;
		return;
	}

	glLineWidth(2.0);
	glColor3f(1.0f, 0.0f, 1.0f);
  
	glBegin(GL_LINE_STRIP);

	GLfloat firstPtX, firstPtY;

	VertexIterator vi = CGAL_Poly.vertices_begin();
	firstPtX = vi->x();
	firstPtY = vi->y();

	for (vi == CGAL_Poly.vertices_begin(); vi != CGAL_Poly.vertices_end(); ++vi)
           glVertex2i(vi->x(),vi->y()); 

    glVertex2i(firstPtX, firstPtY);
    glEnd();
    glFlush();
}

void displayPolygonBoundary()
{
	// Menu option 1 = draw original polygon's boundary

	if (polyInitialized == false)
	{
		cout << "Error! Polygon has not yet been read from file poly.dat." << endl;
		return;
	}

	glLineWidth(2.0);
	glColor3f(1.0f, 0.0f, 0.0f);
  
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < N; i++)
	{
      glVertex2i(vertices[i][0],vertices[i][1]);
	}
	glVertex2i(vertices[0][0],vertices[0][1]);
    glEnd();
    glFlush();
}

void displayPolygonVertices()
{
	// Continuation of menu option 1 = draw original polygon's vertices

	if (polyInitialized == false)
	{
		cout << "Error! Polygon has not yet been read from file poly.dat." << endl;
		return;
	}

	glPointSize(3.0);
	glColor3f(0.0f, 1.0f, 0.0f);

	glBegin(GL_POINTS);
	for (int i = 0; i < N; i++)
	{
      glVertex2i(vertices[i][0],vertices[i][1]);
	}
    glEnd();
    glFlush();
}

void yMonotonePartition()
{
   // Menu item 2 = y-monotone partition (de Berg et al. O(NlogN))
   // Much of this code is from CGAL 2D polygon partition code.

	if (polyInitialized == false)
	{
		cout << "Error! Polygon has not yet been read from file poly.dat." << endl;
		return;
	}

   Polygon_list partition_polys; // list of polygons representing y-monotone partition of p

   CGAL::y_monotone_partition_2(p.vertices_begin(),
                                p.vertices_end(),
                                std::back_inserter(partition_polys));

   cout << endl << "Polygons of y-monotone partition:" << endl;

   std::list<Polygon_2>::const_iterator   poly_it;
   for (poly_it = partition_polys.begin(); poly_it != partition_polys.end();
        poly_it++)
   {
      assert(CGAL::is_y_monotone_2((*poly_it).vertices_begin(),
                                   (*poly_it).vertices_end()));

	   // Traverse and print out the vertices of this polygon of the partition.
  
	  cout << endl << "Polygon of y-monotone partition:" << endl;
      for (VertexIterator vi = (*poly_it).vertices_begin(); vi != (*poly_it).vertices_end(); ++vi)
	   {
             cout << "vertex " << " = " << *vi << endl;
	   }

	  // Use OpenGL to display this polygon of the partition.
	  displayCGAL_Polygon(*poly_it);

      cout << endl;
   }

   assert(CGAL::partition_is_valid_2(p.vertices_begin(),
                                     p.vertices_end(),
                                     partition_polys.begin(),
                                     partition_polys.end()));
}

void optimalDecomposition()
{
   // Menu option 3 = optimal convex decomposition (Greene dynamic programming O(N^4))
   // Much of this code is from CGAL 2D polygon partition code.

	if (polyInitialized == false)
	{
		cout << "Error! Polygon has not yet been read from file poly.dat." << endl;
		return;
	}

   Polygon_list			 partition_polys;
   Traits                partition_traits;
   Validity_traits       validity_traits;

    CGAL::optimal_convex_partition_2(p.vertices_begin(),
                                    p.vertices_end(),
                                    std::back_inserter(partition_polys),
                                    partition_traits);

   cout << endl << "Polygons of optimal decomposition:" << endl;

   std::list<Polygon_2>::const_iterator   poly_it;
   for (poly_it = partition_polys.begin(); poly_it != partition_polys.end();
        poly_it++)
   {
	   // Traverse and print out the vertices of this polygon of the partition.
  
	  cout << endl << "Polygon of optimal decomposition:" << endl;
      for (VertexIterator vi = (*poly_it).vertices_begin(); vi != (*poly_it).vertices_end(); ++vi)
	   {
             cout << "vertex " << " = " << *vi << endl;
	   }

	  // Use OpenGL to display this polygon of the partition.
	  displayCGAL_Polygon(*poly_it);

      cout << endl;
   }

   assert(CGAL::partition_is_valid_2(p.vertices_begin(),
                                     p.vertices_end(),
                                     partition_polys.begin(),
                                     partition_polys.end(),
                                     validity_traits));
									
}

void approxDecomposition()
{
	// Menu option 4 = approximate convex partition  
	//    (triangulation, then Hertel/Mehlhorn O(NlogN))
	// Much of this code is from CGAL 2D polygon partition code.

	if (polyInitialized == false)
	{
		cout << "Error! Polygon has not yet been read from file poly.dat." << endl;
		return;
	}

   Polygon_list			 partition_polys;
  
   CGAL::approx_convex_partition_2(p.vertices_begin(),
                                   p.vertices_end(),
                                   std::back_inserter(partition_polys));

   cout << endl << "Polygons of approximate decomposition:" << endl;

   std::list<Polygon_2>::const_iterator   poly_it;
   for (poly_it = partition_polys.begin(); poly_it != partition_polys.end();
        poly_it++)
   {
	   // Traverse and print out the vertices of this polygon of the partition.
  
	  cout << endl << "Polygon of approximate decomposition:" << endl;
      for (VertexIterator vi = (*poly_it).vertices_begin(); vi != (*poly_it).vertices_end(); ++vi)
	   {
             cout << "vertex " << " = " << *vi << endl;
	   }

	  // Use OpenGL to display this polygon of the partition.
	  displayCGAL_Polygon(*poly_it);

      cout << endl;
   }

   assert(CGAL::convex_partition_is_valid_2(p.vertices_begin(),
                                            p.vertices_end(),
                                            partition_polys.begin(),
                                            partition_polys.end()));
}

void greeneDecomposition()
{
   // Menu option 5 = greene approximate convex partition 
   //    (y-monotone partition, sweep-line approach) O(NlogN)
   // Much of this code is from CGAL 2D polygon partition code.

	if (polyInitialized == false)
	{
		cout << "Error! Polygon has not yet been read from file poly.dat." << endl;
		return;
	}

   Polygon_list			 partition_polys;
   Traits       partition_traits;
  
   CGAL::greene_approx_convex_partition_2(p.vertices_begin(),
                                          p.vertices_end(),
                                          std::back_inserter(partition_polys),
                                          partition_traits);

   cout << endl << "Polygons of Greene approximate decomposition:" << endl;

   std::list<Polygon_2>::const_iterator   poly_it;
   for (poly_it = partition_polys.begin(); poly_it != partition_polys.end();
        poly_it++)
   {
	   // Traverse and print out the vertices of this polygon of the partition.
  
	  cout << endl << "Polygon of Greene approximate decomposition:" << endl;
      for (VertexIterator vi = (*poly_it).vertices_begin(); vi != (*poly_it).vertices_end(); ++vi)
	   {
             cout << "vertex " << " = " << *vi << endl;
	   }

	  // Use OpenGL to display this polygon of the partition.
	  displayCGAL_Polygon(*poly_it);

      cout << endl;
   }

   assert(CGAL::convex_partition_is_valid_2(p.vertices_begin(),
                                            p.vertices_end(),
                                            partition_polys.begin(),
                                            partition_polys.end(),
                                            partition_traits));
}

void drawPoly()
{
	// Main event handler for menu choice

	if (polyInitialized == true)
	{
	 displayPolygonBoundary();
	 displayPolygonVertices();
	}

	switch (typeMode){
		case -1:
			break;
		case 0: // 0 = read polygon from file poly.dat and initialize both
				// vertices array and CGAL polygon
			inputPolygonFile();
			break;
		case 1: // do nothing (except error checking) because polygon is already displayed above for all cases
				// if polygon has been read from file
			if (polyInitialized == false)
			   cout << "Error! Polygon has not yet been read from file poly.dat." << endl;
			break;
		case 2: // 1 = y-monotone partition
			yMonotonePartition();
			break;
		case 3: // 2 = optimal convex decomposition
			optimalDecomposition();
			break;
		case 4: // 3 = approximate convex partition
			approxDecomposition();
			break;
		case 5: // 4 = greene approximate convex partition
			greeneDecomposition();
			break;
		default:
			exit(0);
	}
}

void myDisplay()
{
  glClear(GL_COLOR_BUFFER_BIT);
  drawPoly();  // Contains switch for different menu choices
  glFlush();
}

void mainMenu(int id)
{
	typeMode = id;
	if (typeMode > 5) exit(0);
	glutPostRedisplay();
}

void createMenu()
{
   // OpenGL menu setup

   glutCreateMenu(mainMenu);
   glutAddMenuEntry("Read Polygon from File",0);
   glutAddMenuEntry("Draw Polygon",1);
   glutAddMenuEntry("Y-Monotone Partition",2);
   glutAddMenuEntry("Optimal Convex Decomposition",3);
   glutAddMenuEntry("Approximate Convex Partition", 4);
   glutAddMenuEntry("Greene Approximate Convex Partition", 5);
   glutAddMenuEntry("Exit",6);
   glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void init()
{
   // For openGL viewing transformation (orthographic)

   glClearColor(0.0, 0.0, 1.0, 0.0);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluOrtho2D(0.0, ww, 0.0, wh);
   glutPostRedisplay();
}

void reshape(int w, int h)
{
	// For openGL windowing

	glViewport(0, 0, (GLsizei) w, (GLsizei) h);
	ww = w;
	wh = h;
}

int main(int argc, char **argv)
{
   glutInit(&argc, argv);
   glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
   glutInitWindowSize(ww, wh);
   glutInitWindowPosition(0,0);
   glutCreateWindow("91.504 2D Polygon Partitioning");
   glutDisplayFunc(myDisplay);
   glutReshapeFunc(reshape);
   init();
   createMenu();

   glutMainLoop();
   return 0;
}
