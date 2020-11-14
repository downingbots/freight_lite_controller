//
// simple running average (mean and median) of samples.
// from https://gist.github.com/martinmoene/0205cd59c8e1dfbbe235
//
/*
  USAGE:
     running_average<int,long> av( RUNLENGTH );
     av.add( x );
     rmean = av.rolling_mean();
*/

#include <algorithm>
#include <cassert>
#include <numeric>
#include <vector>

#ifndef  RUNLENGTH
# define RUNLENGTH 10
#endif

// Warning: Don't obtain mean before a sample has been added

template< typename T, typename S = double >
class RollingMeanAcc
{
public:
    typedef T value_type;
    typedef S sum_type;
    typedef std::vector<T> buffer_type;
    typedef typename buffer_type::size_type size_type;


    RollingMeanAcc( size_type N )
    : index( 0 ), capacity( N ) 
    { 
        assert( N > 0 );
        buffer.reserve( N ); 
    }
    
    void reset()
    {
      index = 0;
      buffer.clear();
    }

    void add( value_type const & v )
    {
        if ( buffer.size() < capacity )
        {
            buffer.push_back( v );
        }
        else
        {
            buffer[ index ] = v;
        }
        
        ++index; index %= capacity;
    }

    sum_type rolling_mean() const 
    {
        assert( buffer.size() > 0 );

        return std::accumulate( begin(buffer), end(buffer), sum_type() ) 
             / static_cast<int>( buffer.size() );
    }

private:
    size_type index;
    size_type capacity;
    buffer_type buffer;
};

typedef  RollingMeanAcc<size_t> RollingMeanAcc_t;
