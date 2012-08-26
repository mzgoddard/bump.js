// load: bump.js
// load: LinearMath/Vector3.js
// load: LinearMath/Transform.js
// load: BulletCollision/CollisionShapes/PolyhedralConvexShape.js

(function( window, Bump ) {
  var EPSILON = Math.pow( 2, -52 ),
      tmpV1 = Bump.Vector3.create(),
      tmpV2 = Bump.Vector3.create(),
      tmpV3 = Bump.Vector3.create(),
      tmpV4 = Bump.Vector3.create(),
      tmpT1 = Bump.Transform.create();

  Bump.ConvexHullShape = Bump.type({
    parent: Bump.PolyhedralConvexAabbCachingShape,

    init: function ConvexHullShape( points, numPoints, stride ) {
      points = points || null;
      numPoints = typeof numPoints === 'number' ? numPoints : points && points.length || 0;
      stride = stride || 0;

      this._super();

      this.shapeType = Bump.BroadphaseNativeTypes.CONVEX_HULL_SHAPE_PROXYTYPE;

      this.unscaledPoints = [];
      Bump.resize( this.unscaledPoints, numPoints, Bump.Vector3.create() );

      // TODO: support arraybuffer initialization?
      if ( points ) {
        if ( Array.isArray( points ) ) {
          if ( typeof points[ 0 ] === 'number' ) {
            for ( var i = 0; i < numPoints; ++i ) {
              this.unscaledPoints[ i ].x = points[ i * 3 ];
              this.unscaledPoints[ i ].y = points[ i * 3 + 1 ];
              this.unscaledPoints[ i ].z = points[ i * 3 + 2 ];
            }
          } else {
            Bump.Assert( false );
          }
        }
      }

      this.recalcLocalAabb();
    },

    members: {
      clone: function( dest ) {
        dest = dest || Bump.ConvexHullShape.create();
        dest = this._super( dest );

        return dest;
      },

      getUnscaledPoints: function() {
        return this.unscaledPoints;
      },

      getScaledPoint: function( i, dest ) {
        return this.unscaledPoints[ i ].multiplyVector( this.localScaling, dest );
      },

      getNumPoints: function() {
        return this.unscaledPoints.length;
      },

      setLocalScaling: function( scaling ) {
        this.localScaling.assign( scaling );
        this.recalcLocalAabb();
      },

      addPoint: function( point ) {
        this.unscaledPoints.push( point.clone() );
        this.recalcLocalAabb();
      },

      localGetSupportingVertexWithoutMargin: function( vec, dest ) {
        var supVec = dest || Bump.Vector3.create( 0, 0, 0 );
        var newDot, maxDot = -Infinity;

        for ( var i = 0; i < this.unscaledPoints.length; ++i )
        {
          var vtx = this.unscaledPoints[ i ].multiplyVector( this.localScaling, tmpV1 );

          newDot = vec.dot( vtx );
          if ( newDot > maxDot )
          {
            maxDot = newDot;
            supVec.assign( vtx );
          }
        }
        return supVec;
      },

      batchedUnitVectorGetSupportingVertexWithoutMargin: function( vectors, supportVerticesOut, numVectors ) {
        var newDot, i, j;
        //use 'w' component of supportVerticesOut?
        // {
          for ( i = 0; i < numVectors; ++i ) {
            supportVerticesOut[ i ][ 3 ] = -Infinity;
          }
        // }
        for ( i = 0; i < this.unscaledPoints.length; ++i ) {
          var vtx = this.getScaledPoint( i, tmpV1 );

          for ( j = 0; j < numVectors; ++j ) {
            var vec = vectors[ j ];

            newDot = vec.dot( vtx );
            if ( newDot > supportVerticesOut[ j ][ 3 ] ) {
              //WARNING: don't swap next lines, the w component would get overwritten!
              supportVerticesOut[ j ].assign( vtx );
              supportVerticesOut[ j ][ 3 ] = newDot;
            }
          }
        }
      }
    },

    localGetSupportingVertex: function( vec, dest ) {
      var supVertex = this.localGetSupportingVertexWithoutMargin( vec, dest );

      if ( this.getMargin() !== 0 ) {
        var vecnorm = vec.clone( tmpV1 );
        if ( vecnorm.length2() < ( Bump.SIMD_EPSILON * Bump.SIMD_EPSILON ) ) {
          vecnorm.setValue( -1, -1, -1 );
        } 
        vecnorm.normalize();
        supVertex += this.getMargin() * vecnorm;
      }

      return supVertex;
    },

    getNumVertices: function() {
      return this.unscaledPoints.length;
    },

    getNumEdges: function() {
      return this.unscaledPoints.length;
    },

    getEdge: function( i, pa, pb ) {
      var index0 = i % this.unscaledPoints.length;
      var index1 = ( i + 1 ) % this.unscaledPoints.length;
      // pa = 
      this.getScaledPoint( index0, pa );
      // pb =
      this.getScaledPoint( index1, pb );
    },

    getVertex: function( i, vtx ) {
      // vtx = 
      this.getScaledPoint( i, vtx );
    },

    getNumPlanes: function() {
      return 0;
    },

    // getPlane: function( a, b, i ) {
    //   Bump.Assert(0);
    // },
    // 
    // isInside: function( pt, value ) {
    //   Bump.Assert(0);
    //   return false;
    // },

    project: function( trans, dir, minRef, maxRef ) {
      // #if 1
      var min = Infinity;
      var max = -Infinity;
      var witnesPtMin = tmpV1;
      var witnesPtMax = tmpV2;

      var numVerts = this.unscaledPoints.length;
      for( var i = 0; i < numVerts; ++i ) {
        var vtx = this.unscaledPoints[ i ].multiplyVector( this.localScaling, tmpV3 );
        var pt = trans.multiplyVector( vtx, tmpV3 );
        var dp = pt.dot( dir );
        if ( dp < min ) {
          min = dp;
          witnesPtMin.assign( pt );
        }
        if ( dp > max ) {
          max = dp;
          witnesPtMax.assign( pt );
        }
      }
      // #else
      //   btVector3 localAxis = dir*trans.getBasis();
      //   btVector3 vtx1 = trans(localGetSupportingVertex(localAxis));
      //   btVector3 vtx2 = trans(localGetSupportingVertex(-localAxis));
      // 
      //   min = vtx1.dot(dir);
      //   max = vtx2.dot(dir);
      // #endif

      if ( min > max ) {
        var tmp = min;
        min = max;
        max = tmp;
      }

      minRef.value = min;
      maxRef.value = max;
    }
  });
})( this, this.Bump );
