nclude <stdio.h>
#include <stdlib.h>
#include "vec3fv.hpp"

#define ARRAY_LEN(a) int( sizeof(a) / sizeof((a)[0]) )

//----------------------------------------------------------------------------
//  A few vector routines that weren't provided

static float DotProd4fv(const float a[4], const float b[4])
{
  return( a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3] );
}

static void ScalarMult4fv(float c[4], const float a[4], const float s) // c=a*s
{
  c[0] = a[0] * s;
  c[1] = a[1] * s;
  c[2] = a[2] * s;
  c[3] = a[3] * s;
}

static void Add4fv(float c[4], const float a[4], const float b[4]) // c = a + b
{
  c[0] = a[0] + b[0];
  c[1] = a[1] + b[1];
  c[2] = a[2] + b[2];
  c[3] = a[3] + b[3];
}

static void Subtract4fv(float c[4], const float a[4], const float b[4]) // c = a - b
{
  c[0] = a[0] - b[0];
  c[1] = a[1] - b[1];
  c[2] = a[2] - b[2];
  c[3] = a[3] - b[3];
}

static void Copy4fv(float A[4], const float B[4]) // A=B
{
  A[0]=B[0];
  A[1]=B[1];
  A[2]=B[2];
  A[3]=B[3];
}

static void Vert4fvToStr( const float v[4], char s[80] )
{
  sprintf( s, "[%.2g,%.2g,%.2g,%.2g]", v[0], v[1], v[2], v[3] );
}

//----------------------------------------------------------------------------

class APlaneBoundary4D {
  //  A Boundary is an object of infinite extent which has two properties:
  //    1) A concept of a point being inside or outside the boundary
  //    2) A concept of a boundary-line intersection at exactly one point
  //       or not, for any line.
  //  Examples of a Boundary include a line in 2D and a plane in 3D.
  //
  //  APlaneBoundary4D is a 3D plane defined in 4D homogeneous
  //  coordinates.  It's defined by the standard [A,B,C,D] plane equation
  //  coefficients.  These should be signed so that the "inside" of the
  //  region is where the plane equation evaluates to positive.  BTW, 
  //  APlaneBoundary4D's methods accept simple 4D homogenous points
  //  (e.g. [1,0,0,1])
  //  
  //  "What?" you say!  Yes, a 3D plane in 4D.  Let's return to basics.
  //  Using Blinn's convention of upper-case letters for 3D affine coords
  //  and lower-case letters for homogeneous coordinates, we start with
  //  the basic 3D plane equation:
  //
  //            A*X + B*Y + C*Z + D = 0
  //
  //  Now we plug in homogenous coordinates:
  //
  //            A*(x/w) + B*(y/w) + C*(z/w) + D = 0
  //      aka   A*x + B*y + C*z + D*w = 0
  //
  //  Simple.
  //
  //  So if we take the std 3D plane equation coefficients and
  //  multiply (dot product) by a homogenous 4D point, we've actually
  //  evaluated the plane equation just as we did in 3D by plugging in for
  //  X,Y,Z.  If the result is 0, the point is on the plane.  And if the
  //  result is not 0, then its sign determines which side of the plane
  //  it's on.
  //
  //  We adopt the convention for this class that + means inside and -
  //  means outside (i.e. A*x + B*y + C*z + D*w > 0 means inside, so
  //  sign your [A,B,C,D] coefficients accordingly!)
  //
  //  NOTE:  In Python it's trivial to derive APlaneBoundary4D off a common 
  //         base class "Boundary", and have our PolygonClip rtn be
  //         completely general and work on 2D, 3D, and 4D Boundary objects
  //         equally as well.  In C++, the language's rigidity make that
  //         more trouble than it's worth.  So no base class and no
  //         generic clipper.

  float A,B,C,D;                         // A*x + B*y + C*z + D*w = 0

public:
  APlaneBoundary4D( float _A, float _B, float _C, float _D )
    : A(_A), B(_B), C(_C), D(_D) {}
  
  int PointIsInside( const float pt[4] ) const;
  void LineIntersect( const float pt1[4], const float pt2[4], 
                      int *found, float intersect_pt[4], float &t_val ) const;
  void Get( float coeff[4] ) const;
};


int APlaneBoundary4D::PointIsInside( const float pt[4] ) const
  //  With our definition of a plane, this is trivial.  Evaluate the
  //  plane equation at the point and if the answer is >0, the point is
  //  inside.
{
  return A*pt[0] + B*pt[1] + C*pt[2] + D*pt[3] > 0;
}

void APlaneBoundary4D::LineIntersect( const float pt1[4], const float pt2[4], 
                                     int *found, float intersect_pt[4],
                                     float &t_val ) const
  //  This is a plane-line intersection.  Similar to the technique Blinn
  //  outlines in Down the Graphics Pipeline, Chapter 13.  Plug the
  //  parametric representation of the line:
  //  
  //                 p(t) = (p2-p1)*t + p1
  //  
  //  into the 4D homogeneous plane equation:
  //  
  //                 A*x + B*y + C*z + D*w = 0
  //  
  //  and solve for t:
  //  
  //                 A*x + B*y + C*z + D*w    = 0
  //                 [A,B,C,D] * [x,y,z,w]    = 0
  //                 P * p(t)                 = 0       P = [A,B,C,D]
  //                 P * [ (p2-p1)*t + p1 ]   = 0
  //                 P * [(p2-p1)*t] + P * p1 = 0
  //                 t = (-P * p1) / (P * p2 - P * p1))
  //                 t = (-P * p1) / (P * p2 - P * p1))
  //  
  //  *found will return 0 if the defined line does not intersect
  //  the boundary or is contained in the boundary.  Else *found = 1,
  //  intersect_pt contains the 4D intersection point, and t_val contains
  //  the t value of the intersection point ((0.0 == pt1) --> (1.0 == pt2))
{
  float P[4] = { A, B, C, D };
  float t_denom = DotProd4fv( P, pt2 ) - DotProd4fv( P, pt1 );

  // If t_denom is 0, then the line is parallel to the boundary or
  //   is contained in the boundary.  In both cases, we treat this as
  //   "no point intersection", so return !found.
  if ( t_denom == 0 )
    *found = 0;
  else {
    float v1[4], v2[4];

    // Compute line equation parameter, and find the intersection
    //   point by plugging it in
    t_val = -DotProd4fv( P, pt1 ) / t_denom;

    // Since we're not using classes for vectors and C++ doesn't support
    // vector arithmetic natively, this looks perverse.  Here's what we're
    // really doing:  evaluating the 4D line equation: p(t) = (p2-p1)*t + p1
    Subtract4fv( v1, pt2, pt1 );
    ScalarMult4fv( v2, v1, t_val );
    Add4fv( intersect_pt, v2, pt1 );
      
    *found = 1;
  }
}

void APlaneBoundary4D::Get( float coeff[4] ) const
{
  coeff[0] = A,
  coeff[1] = B,
  coeff[2] = C,
  coeff[3] = D;
}

static void InterpolateColor( float color1[3],
                              float color2[3],
                              float t,
                              float color[3] )
  //  Interpolate a color that is 100*t percent between color1 and color2
  //    (t == 0.0 --> color1; t == 1.0 --> color2).
{
  float delta[3], tmp[3];
  
  // color = (color2-color1)*t + color1
  Subtract3fv( delta, color2, color1 );
  ScalarMult3fv( tmp, delta, t );
  Add3fv( color, tmp, color1 );
}

//---------------------------------------------------------------------------

void PolygonClipToPlane( const float             vertices[][4],
                         const float             vertex_colors[][3],
                         int                     num_vertices,
                         const APlaneBoundary4D &boundary,
                         float                  (*&out_vertices)[4],
                         float                  (*&out_vertex_colors)[3],
                         int                    &num_out_vertices,
                         int                     debug = 0 )
  //  Sutherland-Hodgman polygon clipping derived from F&vD's description.
  //    This is just one step in the process where we clip against
  //    a single boundary plane in 4D.
  //
  //  vertices - 4D points which are "reasonable" 4D points.  For
  //             example, the result of passing 4D points where w=1 
  //             through the standard OpenGL perspective transformation.
  //  num_vertices - length of vertices, in points
  //  boundary     - a plane boundary
  //  out_vertices - output vertex buffer; caller must free
  //  num_out_vertices - output length of out_vertices and out_vertex_colors
  //  debug                - set to 1 for debug prints
{
  float (*in        )[4];
  float (*in_colors )[3];
  float (*out       )[4];
  float (*out_colors)[3];
  float intersect_pt   [4];
  float intersect_color[3];
  float t;
  int   out_len, max_out_len, pt1, pt2, j, found;

  if ( num_vertices <= 2 ) {
    fprintf( stderr, "Bad polygon given to PolygonClip4D.  "
                     "Must have more than 2 vertices." );
    exit(1);
  }

  // For a convex polygon, we'll end up with a poly that has "at most"
  //   one more point.  Though for concave, we may have many more.
  //   *2 is a safe upper limit.
  //max_out_len = num_vertices + 1;
  max_out_len       = num_vertices * 2;
  out_vertices      = (float (*)[4]) new float[ max_out_len * 4 ];
  out_vertex_colors = (float (*)[3]) new float[ max_out_len * 3 ];
  out        = out_vertices;
  out_colors = out_vertex_colors;
  out_len   = 0;
  in        = (float (*)[4]) vertices;
  in_colors = (float (*)[3]) vertex_colors;

  pt1 = (num_vertices - 1);

  for ( j = 0; j < num_vertices; j++ ) {
    pt2 = j;
    if (debug) {
      float plane[4];
      char pt1_str[80], pt2_str[80], plane_str[80];

      Vert4fvToStr( in[ pt1 ], pt1_str );
      Vert4fvToStr( in[ pt2 ], pt2_str );
      boundary.Get( plane );
      Vert4fvToStr( plane, plane_str );

      printf( "CLIP: %s -> %s against Plane %s\n", 
              pt1_str, pt2_str, plane_str );
    }
          
    if ( boundary.PointIsInside( in[ pt2 ] ) )    //  Cases 1 & 4

      if ( boundary.PointIsInside( in[ pt1 ] ) ){ //  Case 1: both inside
        if ( debug ) printf( "      CASE 1: Both in\n" );
        if ( out_len >= max_out_len ) {
          fprintf( stderr, "Clip buffer too small.\n");
          exit(1);
        }
        Copy4fv( out       [ out_len ], in       [ pt2 ] );
        Copy3fv( out_colors[ out_len ], in_colors[ pt2 ] );
        out_len++;
      }
      else {
        if ( debug ) printf( "      CASE 4: out to in\n" );
        boundary.LineIntersect( in[ pt1 ], in[ pt2 ], &found, intersect_pt, t);
        if ( !found ) {
          fprintf( stderr, 
                   "Failed to find intersection point (shouldn't happen)\n" );
          exit(1);
        }
        InterpolateColor( in_colors[ pt1 ], in_colors[ pt2 ], t,
                          intersect_color );

        if ( debug ) {
          char i_str[80];
          Vert4fvToStr( intersect_pt, i_str );
          printf( "      Intersect = %s\n", i_str );
        }
        if ( out_len >= max_out_len-1 ) {
          fprintf( stderr, "Clip buffer too small.\n");
          exit(1);
        }
        Copy4fv( out       [ out_len   ], intersect_pt );
        Copy4fv( out       [ out_len+1 ], in[ pt2 ] );
        Copy3fv( out_colors[ out_len   ], intersect_color );
        Copy3fv( out_colors[ out_len+1 ], in_colors[ pt2 ] );
        out_len += 2;
      }
    else {                                        //  Cases 2 & 3 (3 is a noop)

      if ( boundary.PointIsInside( in[ pt1 ] ) ){ //  Case 2: s in -> p out
        if ( debug ) printf( "      CASE 2: in to out\n" );
        boundary.LineIntersect( in[ pt1 ], in[ pt2 ], &found, intersect_pt, t);
        if ( !found ) {
          fprintf( stderr, 
                   "Failed to find intersection point (shouldn't happen)\n" );
          exit(1);
        }
        InterpolateColor( in_colors[ pt1 ], in_colors[ pt2 ], t,
                          intersect_color );

        if ( debug ) {
          char i_str[80];
          Vert4fvToStr( intersect_pt, i_str );
          printf( "      Intersect = %s\n", i_str );
        }
        if ( out_len >= max_out_len ) {
          fprintf( stderr, "Clip buffer too small.\n");
          exit(1);
        }
        Copy4fv( out       [ out_len ], intersect_pt    );
        Copy3fv( out_colors[ out_len ], intersect_color );
        out_len++;
      }
      else
        if ( debug ) printf( "      CASE 3: both out\n" );
    }
    
    pt1 = pt2;                                    //  On to next vertex pair
  }

  num_out_vertices = out_len;
}

//---------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Clips a 4D homogeneous polygon defined by the packed array of float
// InPts to the viewing frustum defined by w components of the verts. 
// 
// The clipped polygon is put in OutPts (which must be a different location
// than InPts) and the number of vertices in the clipped polygon is
// returned. 
//
// InPts must have NumInPts*4 floats (enough to contain poly). 
//
// Regular orthographic NDC clipping can be achieved by making the w
// coordinate of each point be 1. OutPts will be allocated and return
// filled with the clipped polygon.  
//
// Each vertex has an associated RGB floating point triple that must be
// correctly interpolated to the clip boundaries.
//----------------------------------------------------------------------------

int ClipPolyToFrustum_SMOOTH(float *InPts, float *InPtColors, 
                             const int NumInPts, 
                             float* &OutPts, float* &OutPtColors)
{
  // 3D X,Y,Z (-1,1) clipping planes in 4D
  APlaneBoundary4D boundary_list[] = {
    APlaneBoundary4D( -1, 0, 0, 1 ),   // X =  1 (-x+w=0; X< 1 is inside)
    APlaneBoundary4D(  1, 0, 0, 1 ),   // X = -1 ( x+w=0; X>-1 is inside)
    APlaneBoundary4D(  0,-1, 0, 1 ),   // Y =  1 (-y+w=0; Y< 1 is inside)
    APlaneBoundary4D(  0, 1, 0, 1 ),   // Y = -1 ( y+w=0; Y>-1 is inside)
    APlaneBoundary4D(  0, 0,-1, 1 ),   // Z =  1 (-z+w=0; Z< 1 is inside)
    APlaneBoundary4D(  0, 0, 1, 1 ) }; // Z = -1 ( z+w=0; Z>-1 is inside)

  int     free_in_pts     = 0;
  float   (*in_pts   )[4] = (float (*)[4]) InPts;
  float   (*in_colors)[3] = (float (*)[3]) InPtColors;
  int     in_len      = NumInPts;
  float (*out_pts   )[4];
  float (*out_colors)[3];
  int     out_len;
  int     b;
  int     debug = 0;

  // Clip to all plane boundaries in series (this clipper isn't pipelined)
  for ( b = 0; b < ARRAY_LEN( boundary_list ); b++ ) {
    PolygonClipToPlane( in_pts, in_colors, in_len, boundary_list[ b ],
                        out_pts, out_colors, out_len, debug );
    if ( free_in_pts ) {
      delete [] (float *) in_pts;
      delete [] (float *) in_colors;
    }

    in_pts    = out_pts;
    in_colors = out_colors;
    in_len    = out_len;
    free_in_pts = 1;

    //  If we clipped away all the vertices (no poly left), we're done
    if ( out_len == 0 )
      break;
  }

  if ( debug )
    printf( "----------------\n" );

  OutPts      = (float *) out_pts;
  OutPtColors = (float *) out_colors;
  return(out_len);
}
