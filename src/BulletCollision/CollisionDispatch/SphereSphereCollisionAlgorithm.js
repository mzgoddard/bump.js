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

  Bump.SphereSphereCollisionAlgorithm = Bump.type({
    parent: Bump.ActivatingCollisionAlgorithm,

    init: function SphereSphereCollisionAlgorithm( ci ) {
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
        dest = dest || Bump.SphereSphereCollisionAlgorithm.create();

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

      processCollision: function( col0Wrap, col1Wrap, dispatchInfo, resultOut ) {
        if ( this.manifoldPtr === null ) {
          return;
        }

        resultOut.setPersistentManifold( this.manifoldPtr );

        var sphere0 = col0Wrap.getCollisionShape();
        var sphere1 = col1Wrap.getCollisionShape();

        var diff =
          col0Wrap.getWorldTransform().getOrigin().subtract(
            col1Wrap.getWorldTransform().getOrigin(), getVector3() );
        var len = diff.length();
        var radius0 = sphere0.getRadius();
        var radius1 = sphere1.getRadius();

        // #ifdef CLEAR_MANIFOLD
        // this.manifoldPtr.clearManifold(); //don't do this, it disables warmstarting
        // #endif

        ///iff distance positive, don't generate a new contact
        if ( len > ( radius0 + radius1 ) ) {
          delVector3( diff );

          // #ifndef CLEAR_MANIFOLD
          resultOut.refreshContactPoints();
          // #endif //CLEAR_MANIFOLD
          return;
        }
        ///distance (negative means penetration)
        var dist = len - ( radius0 + radius1 );

        var normalOnSurfaceB = getVector3().setValue( 1, 0, 0 );
        if ( len > Bump.SIMD_EPSILON ) {
          normalOnSurfaceB = diff.divideScalar( len, normalOnSurfaceB );
        }

        ///point on A (worldspace)
        ///btVector3 pos0 = col0->getWorldTransform().getOrigin() - radius0 * normalOnSurfaceB;
        ///point on B (worldspace)
        var pos1 = normalOnSurfaceB.multiplyScalar( radius1, getVector3() );
        pos1.addSelf(
          col1Wrap.getWorldTransform().getOrigin() );

        /// report a contact. internally this will be kept persistent, and contact reduction is done

        resultOut.addContactPoint( normalOnSurfaceB, pos1, dist );

        // #ifndef CLEAR_MANIFOLD
        resultOut.refreshContactPoints();
        // #endif //CLEAR_MANIFOLD

        delVector3( normalOnSurfaceB, pos1, diff );

        // var sphereObjWrap = this.swapped? body1Wrap : body0Wrap;
        // var boxObjWrap = this.swapped? body0Wrap : body1Wrap;
        // 
        // var pOnBox = getVector3();
        // 
        // var normalOnSurfaceB = getVector3();
        // var penetrationDepthRef = { value: 0 };
        // var sphereCenter = sphereObjWrap.getWorldTransform().getOrigin();
        // var sphere0 = sphereObjWrap.getCollisionShape();
        // var radius = sphere0.getRadius();
        // var maxContactDistance = this.manifoldPtr.getContactBreakingThreshold();
        // 
        // resultOut.setPersistentManifold( this.manifoldPtr );
        // 
        // if ( this.getSphereDistance( boxObjWrap, pOnBox, normalOnSurfaceB, penetrationDepthRef, sphereCenter, radius, maxContactDistance ) )
        // {
        //   /// report a contact. internally this will be kept persistent, and contact reduction is done
        //   resultOut.addContactPoint( normalOnSurfaceB, pOnBox, penetrationDepthRef.value );
        // }

        // if ( this.ownManifold )
        // {
        //   resultOut.refreshContactPoints();
        // }
      },

      calculateTimeOfImpact: function() {
        // Not yet.
        return 1;
      },

      getSphereDistance: function( boxObjWrap, pointOnBox, normal, penetrationDepthRef, sphereCenter, fRadius, maxContactDistance ) {

        var boxShape = boxObjWrap.getCollisionShape();
        var boxHalfExtent = boxShape.getHalfExtentsWithoutMargin();
        var boxMargin = boxShape.getMargin();
        penetrationDepthRef.value = 1.0;

        // convert the sphere position to the box's local space
        var m44T = boxObjWrap.getWorldTransform();
        var sphereRelPos = getVector3();
        m44T.invXform( sphereCenter, sphereRelPos );

        // Determine the closest point to the sphere center in the box
        var closestPoint = sphereRelPos.clone( getVector3() );
        closestPoint.x = ( Math.min( boxHalfExtent.x, closestPoint.x ) );
        closestPoint.x = ( Math.max( -boxHalfExtent.x, closestPoint.x ) );
        closestPoint.y = ( Math.min( boxHalfExtent.y, closestPoint.y ) );
        closestPoint.y = ( Math.max( -boxHalfExtent.y, closestPoint.y ) );
        closestPoint.z = ( Math.min( boxHalfExtent.z, closestPoint.z ) );
        closestPoint.z = ( Math.max( -boxHalfExtent.z, closestPoint.z ) );
        
        var intersectionDist = fRadius + boxMargin;
        var contactDist = intersectionDist + maxContactDistance;
        normal = sphereRelPos.subtract( closestPoint, normal );

        //if there is no penetration, we are done
        var dist2 = normal.length2();
        if ( dist2 > contactDist * contactDist ) {
          return false;
        }

        var distance;

        //special case if the sphere center is inside the box
        if ( dist2 === 0.0 ) {
          distance = -this.getSpherePenetration( boxHalfExtent, sphereRelPos, closestPoint, normal );
        }
        else //compute the penetration details
        {
          distance = normal.length();
          normal.divideScalarSelf( distance );
        }

        pointOnBox = closestPoint.add( normal.multiplyScalar( boxMargin, pointOnBox ), pointOnBox );
        //      v3PointOnSphere = sphereRelPos - (normal * fRadius);    
        penetrationDepthRef.value = distance - intersectionDist;

        // transform back in world space
        var tmp = m44T.transform( pointOnBox, getVector3() );
        pointOnBox.assign( tmp );
        //      tmp = m44T(v3PointOnSphere);
        //      v3PointOnSphere = tmp;
        tmp = m44T.getBasis().multiplyVector( normal, tmp );
        normal.assign( tmp );

        return true;

      },

      getSpherePenetration: function( boxHalfExtent, sphereRelPos, closestPoint, normal ) {
        //project the center of the sphere on the closest face of the box
        var faceDist = boxHalfExtent.x - sphereRelPos.x;
        var minDist = faceDist;
        closestPoint.x = ( boxHalfExtent.x );
        normal.setValue( 1.0, 0.0, 0.0 );

        faceDist = boxHalfExtent.x + sphereRelPos.x;
        if ( faceDist < minDist )
        {
                minDist = faceDist;
                closestPoint.assign( sphereRelPos );
                closestPoint.x = ( -boxHalfExtent.x );
                normal.setValue( -1.0, 0.0, 0.0 );
        }

        faceDist = boxHalfExtent.y - sphereRelPos.y;
        if ( faceDist < minDist )
        {
                minDist = faceDist;
                closestPoint.assign( sphereRelPos );
                closestPoint.y = ( boxHalfExtent.y );
                normal.setValue( 0.0, 1.0, 0.0 );
        }

        faceDist = boxHalfExtent.y + sphereRelPos.y;
        if ( faceDist < minDist )
        {
                minDist = faceDist;
                closestPoint.assign( sphereRelPos );
                closestPoint.y = ( -boxHalfExtent.y );
                normal.setValue( 0.0, -1.0, 0.0 );
        }

        faceDist = boxHalfExtent.z - sphereRelPos.z;
        if ( faceDist < minDist )
        {
                minDist = faceDist;
                closestPoint.assign( sphereRelPos );
                closestPoint.z = ( boxHalfExtent.z );
                normal.setValue( 0.0, 0.0, 1.0 );
        }

        faceDist = boxHalfExtent.z + sphereRelPos.z;
        if ( faceDist < minDist )
        {
                minDist = faceDist;
                closestPoint.assign( sphereRelPos );
                closestPoint.z = ( -boxHalfExtent.z );
                normal.setValue( 0.0, 0.0, -1.0 );
        }

        return minDist;
      },

      getAllContactManifolds: function( manifoldArray ) {
        if ( this.manifoldPtr && this.ownManifold ) {
          manifoldArray.push( this.manifoldPtr );
        }
      }

    },

    typeMembers: {
      create: function( a, b, c, d ) {
        var ca = Object.create( Bump.SphereSphereCollisionAlgorithm.prototype );
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
            var ca = Bump.SphereSphereCollisionAlgorithm.create( null, ci, body0, body1 );
            ca.swapped = this.swapped;
            return ca;
          }
        }
      })
    }
  });

})( this, this.Bump );
