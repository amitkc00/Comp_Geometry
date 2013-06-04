//
//  Shortest_Path.cpp
//  COMPGEO
//
//  Created by Amit Choudhary on 10/3/12.
//  Copyright (c) 2012 University of Massachusetts Lowell. All rights reserved.
//

//#include <iostream>


///////////

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

// Adapted from Prof. Daniels partition.cpp program file.

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
// 2 = Draw 2-D Convex Hull of given 2-D Polygon Points.
// 3 = Input end points A & B using Mouse Left Button. 
// 4 = Draw Shortest Path between A & B.
//    (triangulation, then Hertel/Mehlhorn O(NlogN))
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

//START NEW
#include <CGAL/convex_hull_2.h>
Polygon_2 poly2; //I want to make it a global value so that it can be accessed outside too.
Polygon_list poly2_list;
void draw_2D_ConvexHull() {
    
    if (polyInitialized == false)
	{
		cout << "Error! Polygon has not yet been read from file poly.dat." << endl;
		return;
	}
    
    Traits partition_traits;
    
    CGAL::convex_hull_2( p.vertices_begin(), p.vertices_end(), std::back_inserter(poly2),partition_traits);
    
    cout<<endl<<"Polygons of Convex Hull:"<<endl;
  
	for (VertexIterator vi = (poly2).vertices_begin(); vi != (poly2).vertices_end(); ++vi)
    {
    	cout << "vertex " << " = " << *vi << endl;
    }

    displayCGAL_Polygon(poly2);
}


GLfloat A_xValue, A_yValue; 
GLfloat B_xValue,B_yValue;
int Flag = 0;
void accept_left_button_command(int btn, int state, int x, int y)
{
	if(btn==GLUT_LEFT_BUTTON && state==GLUT_DOWN && Flag==0) //this is only for left button events
	{
		A_xValue = (float)x;  //capture x mouse position in global
		A_yValue = (float)(wh-y);  //capture y mouse position in global
		glutPostRedisplay(); //Since dislay event is not automatic for mouse action.
		Flag=1;
		cout<<"A_xValue = "<<A_xValue<<" A_yValue = "<<A_yValue<<endl;
	}


	if(btn==GLUT_LEFT_BUTTON && state==GLUT_UP && Flag==1) //this is only for left button events
	{
		Flag=2;
	}
	
	if(btn==GLUT_LEFT_BUTTON && state==GLUT_DOWN && Flag==2) //this is only for left button events
	{
		B_xValue = (float)x;  //capture x mouse position in global
		B_yValue = (float)(wh-y);  //capture y mouse position in global
		glutPostRedisplay(); //Since dislay event is not automatic for mouse action.
		Flag=-1;
		cout<<"B_xValue = "<<B_xValue<<" B_yValue = "<<B_yValue<<endl;
	}
}

void inputAB() 
{
    std::cout<<"Accepted Mouse Inputs"<<endl;
	glutMouseFunc(accept_left_button_command);
    
}



#include <CGAL/enum.h>
void shortestPath() {
	CGAL::Oriented_side side;
    Polygon_2 P_Side, N_Side;
    double N_distance=0.0;
    double P_distance=0.0;
    
    Point_2 a( A_xValue, A_yValue); 
    Point_2 b( B_xValue, B_yValue); 
	
    Traits::Line_2 l(a,b);
    
    P_Side.push_back(Point_2(a));

	for (VertexIterator vi = (poly2).vertices_begin(); vi != (poly2).vertices_end(); ++vi)
	{
		cout << "vertex " << " = " << *vi << endl;
	}


    for (VertexIterator vi = (poly2).vertices_begin(); vi != (poly2).vertices_end(); ++vi)
    {
		side=l.oriented_side(*vi);
		cout<<"SIDE = "<<side<<endl;

        switch (side) {
            case  -1:
                N_Side.push_back(*vi);
                break;
                
            case 1:
                P_Side.push_back(*vi);
                break;
                
            default: //TEST
                exit;
        }
    }
    
    N_Side.push_back(Point_2(b));
    P_Side.push_back(Point_2(b));
    
    
	//NOW PRINT N_SIDE & P_SIDE ARRAY
	for (VertexIterator vi = (N_Side).vertices_begin(); vi != (N_Side).vertices_end(); ++vi)
    {
        cout << "N_Side vertex " << " = " << *vi << endl;
    }
    
    for (VertexIterator vi = (P_Side).vertices_begin(); vi != (P_Side).vertices_end(); ++vi)
    {
        cout << "P_Side vertex " << " = " << *vi << endl;
    }
    
    
	//NOW SORT N_SIDE & P_SIDE ARRAY
	// NOT IMPLEMENTED. For future enchancements. 
    
	//PRINT THE SORTED N_SIDE & P_SIDE ARRAY
    
	double x1, y1,x2,y2;
	double distance=0.0;
	int N_Side_Size = N_Side.size();
	int iter_size=1;
	cout<<"Size of N_Side = "<<N_Side_Size<<endl;
	VertexIterator vi = (N_Side).vertices_begin();

	while(iter_size != N_Side.size()) {
		x1=vi->x();
		y1=vi->y();
		++vi;
		x2=vi->x();
		y2=vi->y();
		iter_size++;
		
		distance=sqrt(pow((x2-x1),2)+pow((y2-y1),2));
        N_distance = N_distance + distance;
	}

	cout<<" Calculated N_distance = "<<N_distance<<endl;
	
	int P_Side_Size = P_Side.size();
	int iter_size2=1;
	cout<<"Size of P_Side = "<<P_Side_Size<<endl;
	VertexIterator vi2 =  (P_Side).vertices_begin();

	while(iter_size2 != P_Side.size()) {
		x1=vi2->x();
		y1=vi2->y();
		++vi2;
		x2=vi2->x();
		y2=vi2->y();
		iter_size2++;
		
		distance=sqrt(pow((x2-x1),2)+pow((y2-y1),2));
        P_distance = P_distance + distance;
	
	}
	cout<<"Calculated P_distance = "<<P_distance<<endl;
    
    if(N_distance<P_distance)
    {
        cout<<"Negative Side of Line AB is having shortest Path";
        displayCGAL_Polygon(N_Side);
    }
    else
    {
        cout<<"Positive Side of Line AB is having shortest Path";
        displayCGAL_Polygon(P_Side);
    }
    
}


//END NEW

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
        case 2:
            draw_2D_ConvexHull();
            break;
        case 3:
            inputAB();
            break;
        case 4:
            shortestPath();
            break;
            //END NEW
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
    
    //START NEW
    glutAddMenuEntry("Draw 2-D Convex Hull", 2);
    glutAddMenuEntry("Select points A & B using Mouse Left Button", 3);
    glutAddMenuEntry("Draw Shortest Path from A to B", 4);
    glutAddMenuEntry("Exit",5);
    //START END
    
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


