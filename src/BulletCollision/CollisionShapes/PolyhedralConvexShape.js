// load: bump.js
// load: LinearMath/Vector3.js
// load: LinearMath/Transform.js
// load: BulletCollision/CollisionShapes/ConvexInternalShape.js

(function( window, Bump ) {
  var EPSILON = Math.pow( 2, -52 ),
      tmpV1 = Bump.Vector3.create(),
      tmpV2 = Bump.Vector3.create(),
      tmpV3 = Bump.Vector3.create(),
      tmpV4 = Bump.Vector3.create(),
      tmpT1 = Bump.Transform.create();

  Bump.PolyhedralConvexShape = Bump.type({
    parent: Bump.ConvexInternalShape,

    init: function PolyhedralConvexShape() {
      this._super();

      this.polyhedron = null;
    },

    members: {
      clone: function( dest ) {
        dest = dest || Bump.PolyhedralConvexShape.create();
        dest = this._super( dest );

        dest.polyhedron = this.polyhedron;
        return dest;
      },

      initializePolyhedralFeatures: function() {
        Bump.Assert( 'Not implemented' && false );
      },

      getConvexPolyhedron: function() {
        return this.polyhedron;
      },

      // Uses the following temporary variables:
      //
      // - `tmpV1`
      // - `tmpV2`
      // - `tmpV3`
      localGetSupportingVertexWithoutMargin: function( vec0, dest ) {
        dest = dest || Bump.Vector3.create();

        var i, supVec = tmpV1.setValue( 0, 0, 0 ),
            maxDot = -Infinity;

        var vec = vec0.clone( tmpV2 ),
            lenSqr = vec.length2();
        if ( lenSqr < 0.0001 ) {
          vec.setValue( 1, 0, 0 );
        } else {
          var rlen = 1 / Math.sqrt( lenSqr );
          vec.multiplyScalarSelf( rlen );
        }

        var vtx = tmpV3,
            newDot;

        for ( i = 0; i < this.getNumVertices(); ++i ) {
          this.getVertex( i, vtx );
          newDot = vec.dot( vtx );
          if ( newDot > maxDot ) {
            maxDot = newDot;
            supVec = vtx.clone( supVec );
          }
        }

        return supVec.clone( dest );
      },

      // Uses the following temporary variables:
      //
      // - `tmpV1`
      batchedUnitVectorGetSupportingVertexWithoutMargin: function( vectors, supportVerticesOut, numVectors ) {
        var i,
            vtx = tmpV1,
            newDot;

        for ( i = 0; i < numVectors; ++i ) {
          supportVerticesOut[i].w = -Infinity;
        }

        for ( var j = 0; j < numVectors; ++j ) {
          var vec = vectors[j];

          for ( i = 0; i < this.getNumVertices(); ++i ) {
            this.getVertex( i, vtx );
            newDot = vec.dot( vtx );
            if ( newDot > supportVerticesOut[j].w ) {
              // **Warning:** Don't swap next lines, the `w` component would get
              // overwritten!
              supportVerticesOut[j] = vtx.clone( supportVerticesOut[j] );
              supportVerticesOut[j].w = newDot;
            }
          }
        }
      },

      // Uses the following temporary variables:
      //
      // - `tmpV1`
      // - `tmpV2`
      // - `tmpV3`
      // - `tmpV4`
      calculateLocalInertia: function( mass, inertia ) {
        var margin = this.getMargin();

        var ident = tmpT1;
        ident.setIdentity();
        var aabbMin = tmpV1, aabbMax = tmpV2;
        this.getAabb( ident, aabbMin, aabbMax );
        var halfExtents = aabbMax
          .subtract( aabbMin, tmpV3 )
          .multiplyScalar( 0.5, tmpV3 );

        var lx = 2 * ( halfExtents.x + margin ),
            ly = 2 * ( halfExtents.y + margin ),
            lz = 2 * ( halfExtents.z + margin ),
            x2 = lx * lx,
            y2 = ly * ly,
            z2 = lz * lz,
            scaledmass = mass * 0.08333333;

        // inertia = scaledmass * ( btVector3( y2 + z2, x2 + z2, x2 + y2 ) );
        inertia = tmpV4
          .setValue( y2 + z2, x2 + z2, x2 + y2 )
          .multiplyScalarSelf( scaledmass )
          .clone( inertia );
      }
    }
  });

  Bump.PolyhedralConvexAabbCachingShape = Bump.type({
    parent: Bump.PolyhedralConvexShape,

    init: function PolyhedralConvexShape() {
      this._super();

      this.localAabbMin = Bump.Vector3.create( 1, 1, 1 );
      this.localAabbMax = Bump.Vector3.create( -1, -1, -1 );
      this.isLocalAabbValid = false;
    },

    members: {
      setCachedLocalAabb: function( aabbMin, aabbMax) {
        this.isLocalAabbValid = true;
        this.localAabbMin.assign( aabbMin );
        this.localAabbMax.assign( aabbMax );
      },

      getCachedLocalAabb: function( aabbMin, aabbMax) {
        Bump.Assert( this.isLocalAabbValid );
        aabbMin.assign( this.localAabbMin );
        aabbMax.assign( this.localAabbMax );
      },

      getNonvirtualAabb: function( trans, aabbMin, aabbMax, margin) {

        //lazy evaluation of local aabb
        Bump.Assert( this.isLocalAabbValid );
        Bump.TransformAabb( this.localAabbMin, this.localAabbMax, margin, trans, aabbMin, aabbMax );
      },

      setLocalScaling: function( scaling ) {
        Bump.ConvexInternalShape.prototype.setLocalScaling.call( this, scaling );
        this.recalcLocalAabb();
      },

      getAabb: function( trans, aabbMin, aabbMax ) {
        this.getNonvirtualAabb( trans, aabbMin, aabbMax, this.getMargin() );
      },

      recalcLocalAabb: function() {
        this.isLocalAabbValid = true;

        // #if 1
        // static const btVector3 _directions[] =
        var _directions =
        [
          Bump.Vector3.create( 1,  0,  0 ),
          Bump.Vector3.create( 0,  1,  0 ),
          Bump.Vector3.create( 0,  0,  1 ),
          Bump.Vector3.create( -1, 0,  0 ),
          Bump.Vector3.create( 0, -1,  0 ),
          Bump.Vector3.create( 0,  0, -1 )
        ];

        // btVector3 _supporting[] =
        var _supporting =
        [
          Bump.Vector3.create( 0, 0, 0 ),
          Bump.Vector3.create( 0, 0, 0 ),
          Bump.Vector3.create( 0, 0, 0 ),
          Bump.Vector3.create( 0, 0, 0 ),
          Bump.Vector3.create( 0, 0, 0 ),
          Bump.Vector3.create( 0, 0, 0 )
        ];

        this.batchedUnitVectorGetSupportingVertexWithoutMargin( _directions, _supporting, 6 );

        for ( var i = 0; i < 3; ++i ) {
          this.localAabbMax[ i ] = _supporting[ i ][ i ] + this.collisionMargin;
          this.localAabbMin[ i ] = _supporting[ i + 3 ][ i ] - this.collisionMargin;
        }

        // #else
        // 
        // for (int i=0;i<3;i++)
        // {
        //   btVector3 vec(btScalar(0.),btScalar(0.),btScalar(0.));
        //   vec[i] = btScalar(1.);
        //   btVector3 tmp = localGetSupportingVertex(vec);
        //   m_localAabbMax[i] = tmp[i];
        //   vec[i] = btScalar(-1.);
        //   tmp = localGetSupportingVertex(vec);
        //   m_localAabbMin[i] = tmp[i];
        // }
        // #endif
      }
    }
  });
})( this, this.Bump );
