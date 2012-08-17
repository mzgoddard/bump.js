// load: bump.js
// load: BulletCollision/CollisionDispatch/ActivatingCollisionAlgorithm.js
// load: BulletCollision/CollisionDispatch/CollisionAlgorithmCreateFunc.js
// load: BulletCollision/CollisionDispatch/BoxBoxDetector.js
// load: BulletCollision/NarrowPhaseCollision/DiscreteCollisionDetectorInterface.js

(function( window, Bump ) {

  var createGetter = function( Type, pool ) {
    return function() {
      return pool.pop() || Type.create();
    };
  };

  var createDeller = function( pool ) {
    return function() {
      for ( var i = 0; i < arguments.length; ++i ) {
        pool.push( arguments[i] );
      }
    };
  };

  var vecPool = [];

  var getVector3 = createGetter( Bump.Vector3, vecPool );
  var delVector3 = createDeller( vecPool );

  // used in getSphereDistance and getSpherePenetration
  var GSDn = [
        Bump.Vector3.create( -1.0, 0.0, 0.0 ),
        Bump.Vector3.create(  0.0, -1.0, 0.0 ),
        Bump.Vector3.create(  0.0, 0.0, -1.0 ),
        Bump.Vector3.create(  1.0, 0.0, 0.0 ),
        Bump.Vector3.create(  0.0, 1.0, 0.0 ),
        Bump.Vector3.create(  0.0, 0.0, 1.0 )
      ];

  Bump.SphereBoxCollisionAlgorithm = Bump.type({
    parent: Bump.ActivatingCollisionAlgorithm,

    init: function SphereBoxCollisionAlgorithm( ci ) {
      this._super( ci );

      this.ownManifold = false;
      this.manifoldPtr = null;
      return this;
    },

    members: {
      initWithManifold: function( mf, ci, obj0, obj1 ) {
        Bump.ActivatingCollisionAlgorithm.prototype.init.apply( this, [ ci, obj0, obj1 ] );
        this.ownManifold = false;
        this.manifoldPtr = mf;

        if ( this.manifoldPtr === null && this.dispatcher.needsCollision( obj0, obj1 ) ) {
          this.manifoldPtr = this.dispatcher.getNewManifold( obj0, obj1 );
          this.ownManifold = true;
        }
        return this;
      },

      clone: function( dest ) {
        dest = dest || Bump.SphereBoxCollisionAlgorithm.create();

        this._super( dest );

        dest.ownManifold = this.ownManifold;
        dest.manifoldPtr = this.manifoldPtr;

        return dest;
      },

      assign: function( other ) {
        this._super( other );

        this.ownManifold = other.ownManifold;
        this.manifoldPtr = other.manifoldPtr;

        return this;
      },

      destruct: function() {
        if ( this.ownManifold ) {
          if ( this.manifoldPtr !== null ) {
            this.dispatcher.releaseManifold( this.manifoldPtr );
          }
        }

        this._super();
      },

      processCollision: function( body0, body1, dispatchInfo, resultOut ) {
        if ( this.manifoldPtr === null ) {
          return;
        }

        var sphereObj = this.swapped ? body1 : body0,
            boxObj = this.swapped ? body0 : body1,
            sphere0 = sphereObj.getCollisionShape(),
            box1 = boxObj.getCollisionShape();

        var normalOnSurfaceB = getVector3();
        var pOnBox = getVector3(), pOnSphere = getVector3();
        var sphereCenter = sphereObj.getWorldTransform().getOrigin();
        var radius = sphere0.getRadius();
  
        var dist = this.getSphereDistance( boxObj, pOnBox, pOnSphere, sphereCenter, radius );

        resultOut.setPersistentManifold( this.manifoldPtr );

        if ( dist < Bump.SIMD_EPSILON )
        {
          pOnBox.subtract( pOnSphere, normalOnSurfaceB ).normalize();

          /// report a contact. internally this will be kept persistent, and contact reduction is done

          resultOut.addContactPoint( normalOnSurfaceB, pOnBox, dist );

        }

        if ( this.ownManifold )
        {
          // if ( this.manifoldPtr.getNumContacts() )
          // {
            resultOut.refreshContactPoints();
          // }
        }

        delVector3( normalOnSurfaceB, pOnBox, pOnSphere );

        // // `refreshContactPoints` is only necessary when using persistent
        // // contact points. Otherwise all points are newly added.
        // if ( this.ownManifold ) {
        //   resultOut.refreshContactPoints();
        // }
      },

      calculateTimeOfImpact: function() {
        // Not yet.
        return 1;
      },

      getSphereDistance: function( boxObj, pointOnBox, v3PointOnSphere, sphereCenter, fRadius ) {

        var margins;
        var bounds = [ null, null ];
        var boxShape = boxObj.getCollisionShape();

        bounds[ 1 ] = boxShape.getHalfExtentsWithoutMargin().clone( getVector3() );
        bounds[ 0 ] = bounds[ 1 ].negate( getVector3() );

        margins = boxShape.getMargin(); //also add sphereShape margin?

        var m44T = boxObj.getWorldTransform();

        var boundsVec = [ null, null ];
        var fPenetration;

        boundsVec[ 0 ] = bounds[ 0 ].clone( getVector3() );
        boundsVec[ 1 ] = bounds[ 1 ].clone( getVector3() );

        var marginsVec = getVector3().setValue( margins, margins, margins );

        // add margins
        bounds[ 0 ].addSelf( marginsVec );
        bounds[ 1 ].subtractSelf( marginsVec );

        /////////////////////////////////////////////////

        // btVector3
        var tmp = getVector3(),
            prel = getVector3(),
            n = GSDn,
            normal = getVector3(),
            v3P = getVector3();
        var fSep = 10000000.0, fSepThis; // btScalar

        // convert  point in local space
        prel = m44T.invXform( sphereCenter, prel );

        var bFound = false;

        v3P.assign( prel );

        for (var i = 0; i < 6; i++ ) {
          var j = i < 3 ? 0 : 1;
          if ( ( fSepThis = ( ( v3P.subtract( bounds[ j ], tmp ).dot( n[ i ] ) ) ) ) > 0.0 ) {
            v3P.subtractSelf( n[ i ].multiplyScalar( fSepThis, tmp ) );
            bFound = true;
          }
        }

        //

        if ( bFound ) {
          bounds[ 0 ].assign( boundsVec[ 0 ] );
          bounds[ 1 ].assign( boundsVec[ 1 ] );

          // normal = ( prel - v3P ).normalize();
          prel.subtract( v3P, normal ).normalize();
          // pointOnBox = v3P + normal * margins;
          v3P.add( normal.multiplyScalar( margins, tmp ), pointOnBox );
          // v3PointOnSphere = prel - normal * fRadius;
          prel.subtract( normal.multiplyScalar( fRadius, tmp ), v3PointOnSphere );

          if ( ( ( v3PointOnSphere.subtract( pointOnBox, tmp ) ).dot( normal ) ) > 0.0 ) {

            delVector3(
              bounds[ 0 ], bounds[ 1 ], boundsVec[ 0 ], boundsVec[ 1 ],
              tmp, normal, prel, v3P
            );

            return 1.0;
          }

          // transform back in world space
          m44T.transform( pointOnBox, tmp );
          pointOnBox.assign( tmp );
          m44T.transform( v3PointOnSphere, tmp );
          v3PointOnSphere.assign( tmp );
          var fSeps2 = ( pointOnBox.subtract( v3PointOnSphere, tmp ) ).length2();

          //if this fails, fallback into deeper penetration case, below
          if ( fSeps2 > Bump.SIMD_EPSILON ) {
            fSep = - Math.sqrt( fSeps2 );
            normal = ( pointOnBox.subtract( v3PointOnSphere, normal ) );
            normal.multiplyScalarSelf( 1.0 / fSep );
          }

          delVector3(
            bounds[ 0 ], bounds[ 1 ], boundsVec[ 0 ], boundsVec[ 1 ],
            tmp, normal, prel, v3P
          );

          return fSep;
        }

        //////////////////////////////////////////////////
        // Deep penetration case

        fPenetration = this.getSpherePenetration(
          boxObj, pointOnBox, v3PointOnSphere, sphereCenter,
          fRadius, bounds[0], bounds[1] );

        bounds[ 0 ].assign( boundsVec[0] );
        bounds[ 1 ].assign( boundsVec[1] );

        delVector3(
          bounds[ 0 ], bounds[ 1 ], boundsVec[ 0 ], boundsVec[ 1 ],
          tmp, normal, prel, v3P
        );

        if ( fPenetration <= 0.0 ) {
          return ( fPenetration - margins );
        } else {
          return 1.0;
        }
      },

      getSpherePenetration: function( boxObj, pointOnBox, v3PointOnSphere, sphereCenter, fRadius, aabbMin, aabbMax ) {

        var bounds = [ null, null ];

        bounds[ 0 ] = aabbMin;
        bounds[ 1 ] = aabbMax;

        var p0 = getVector3(),
            tmp = getVector3(),
            prel = getVector3(),
            n = GSDn,
            normal = getVector3();
        var fSep = -10000000.0, fSepThis;

        // set p0 and normal to a default value to shup up GCC
        // p0.setValue( 0.0, 0.0, 0.0 );
        // normal.setValue( 0.0, 0.0, 0.0 );

        var m44T = boxObj.getWorldTransform();

        // convert  point in local space
        m44T.invXform( sphereCenter, prel );

        ///////////

        for ( var i = 0; i < 6; i++ ) {
          var j = i < 3 ? 0 : 1;
          if ( ( fSepThis = ( ( prel.subtract( bounds[ j ], tmp ) ).dot( n[ i ] ) ) - fRadius ) > 0.0 ) {

            delVector3(
              tmp, normal, prel, p0
            );

            return 1.0;
          }
          if ( fSepThis > fSep ) {
            p0.assign( bounds[ j ] );
            normal.assign( n[ i ] );
            fSep = fSepThis;
          }
        }

        prel.subtract( normal.
          multiplyScalar(
            normal.dot( ( prel.subtract( p0, tmp ) ) ),
            tmp
          ), pointOnBox );
        pointOnBox.add(
          normal.multiplyScalar( fSep, tmp ),
          v3PointOnSphere );

        // transform back in world space
        tmp = m44T.transform( pointOnBox, tmp );    
        pointOnBox.assign( tmp );
        tmp  = m44T.transform( v3PointOnSphere, tmp );
        v3PointOnSphere.assign( tmp );
        // normal = ( pointOnBox.subtract( v3PointOnSphere, normal ) ).normalize();

        delVector3(
          tmp, normal, prel, p0
        );

        return fSep;

      },

      getAllContactManifolds: function( manifoldArray ) {
        if ( this.manifoldPtr && this.ownManifold ) {
          manifoldArray.push( this.manifoldPtr );
        }
      }

    },

    typeMembers: {
      create: function( a, b, c, d ) {
        var ca = Object.create( Bump.SphereBoxCollisionAlgorithm.prototype );
        if ( b === undefined ) {
          return ca.init( a );
        }

        return ca.initWithManifold( a, b, c, d );
      },

      CreateFunc: Bump.type({
        parent: Bump.CollisionAlgorithmCreateFunc,

        init: function CreateFunc() {
          this._super();
        },

        members: {
          CreateCollisionAlgorithm: function( ci, body0, body1 ) {
            var ca = Bump.SphereBoxCollisionAlgorithm.create( null, ci, body0, body1 );
            ca.swapped = this.swapped;
            return ca;
          }
        }
      })
    }
  });

})( this, this.Bump );
