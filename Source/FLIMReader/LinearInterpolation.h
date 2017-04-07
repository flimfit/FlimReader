# include <cstdlib>
# include <iostream>
# include <iomanip>
# include <cmath>
# include <ctime>
# include <cstring>
# include <vector>
# include <assert.h>

template<typename TX, typename U, typename V>
void pwl_interp_1d ( std::vector<TX>& xd, std::vector<U>& yd, std::vector<TX>& xi, std::vector<V>& yi )

//****************************************************************************80
//
//  Purpose:
//
//    PWL_INTERP_1D evaluates the piecewise linear interpolant.
//
//  Discussion:
//
//    The piecewise linear interpolant L(ND,XD,YD)(X) is the piecewise
//    linear function which interpolates the data (XD(I),YD(I)) for I = 1
//    to ND.
//
//  Licensing:
//
//    This code is distributed under the GNU LGPL license.
//
//  Modified:
//
//    22 September 2012
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int ND, the number of data points.
//    ND must be at least 1.
//
//    Input, double XD[ND], the data points.
//
//    Input, double YD[ND], the data values.
//
//    Input, int NI, the number of interpolation points.
//
//    Input, double XI[NI], the interpolation points.
//
//    Output, double PWL_INTERP_1D[NI], the interpolated values.
//
{
  int i;
  int k;
  double t;

  int nd = (int) xd.size();
  assert( xd.size() == yd.size() );

  int ni = (int) xi.size();
  yi.resize( ni );

  for ( i = 0; i < ni; i++ )
  {
    yi[i] = 0.0;
  }

  if ( nd == 1 )
  {
    for ( i = 0; i < ni; i++ )
    {
      yi[i] = yd[0];
    }
    return;
  }

  int klast = 1;
  for ( i = 0; i < ni; i++ )
  {
    if ( xi[i] <= xd[0] )
    {
      t = ( xi[i] - xd[0] ) / ( xd[1] - xd[0] );
      yi[i] = ( 1.0 - t ) * yd[0] + t * yd[1];
    }
    else if ( xd[nd-1] <= xi[i] )
    {
      t = ( xi[i] - xd[nd-2] ) / ( xd[nd-1] - xd[nd-2] );
      yi[i] = ( 1.0 - t ) * yd[nd-2] + t * yd[nd-1];
    }
    else
    {
      for ( k = klast; k < nd; k++ )
      {
        if ( xd[k-1] <= xi[i] && xi[i] <= xd[k] )
        {
          t = ( xi[i] - xd[k-1] ) / ( xd[k] - xd[k-1] );
          yi[i] = ( 1.0 - t ) * yd[k-1] + t * yd[k];
          klast = k;
          break;
        }
      }
    }
  }
  return;
}