/*global Stats:false THREEWrapper:false*/

(function() {
  var stats = new Stats();
  // stats.setMode(1);

  // Align top-left
  stats.domElement.style.position = 'absolute';
  stats.domElement.style.left = '0px';
  stats.domElement.style.top = '0px';

  document.body.appendChild( stats.domElement );

  var collisionConfiguration = Bump.DefaultCollisionConfiguration.create();
  var dispatcher = Bump.CollisionDispatcher.create( collisionConfiguration );
  var overlappingPairCache = Bump.DbvtBroadphase.create();
  var solver = Bump.SequentialImpulseConstraintSolver.create();
  var dynamicsWorld = Bump.DiscreteDynamicsWorld.create( dispatcher, overlappingPairCache, solver, collisionConfiguration );
  dynamicsWorld.setGravity( Bump.Vector3.create( 0, -9.8, 0 ) );
  var effectsCollisionConfiguration = Bump.DefaultCollisionConfiguration.create();
  var effectsDispatcher = Bump.CollisionDispatcher.create( effectsCollisionConfiguration );
  var effectsOverlappingPairCache = Bump.DbvtBroadphase.create();
  var effectsSolver = Bump.SequentialImpulseConstraintSolver.create();
  var effectsWorld = Bump.DiscreteDynamicsWorld.create( effectsDispatcher, effectsOverlappingPairCache, effectsSolver, effectsCollisionConfiguration );

  var collisionShapes = [];

  var groundBody;
  (function( size ) {
    var groundHalfExtents = Bump.Vector3.create( size, size, size );
    var groundBoxShape = Bump.BoxShape.create( groundHalfExtents );
    var groundShape = Bump.CompoundShape.create();

    collisionShapes.push( groundShape );
    collisionShapes.push( groundBoxShape );

    var sizeAndHalf = 1.5 * size;
    [
      Bump.Vector3.create( 0, -sizeAndHalf, 0 ),
      Bump.Vector3.create( 0,  sizeAndHalf, 0 ),
      Bump.Vector3.create( -sizeAndHalf, 0, 0 ),
      Bump.Vector3.create(  sizeAndHalf, 0, 0 ),
      Bump.Vector3.create( 0, 0, -sizeAndHalf ),
      Bump.Vector3.create( 0, 0,  sizeAndHalf )
    ].forEach(function( position ) {
      var localTransform = Bump.Transform.getIdentity();
      localTransform.setOrigin( position );

      groundShape.addChildShape( localTransform, groundBoxShape );
    });

    var groundTransform = Bump.Transform.getIdentity();

    var myMotionState = Bump.DefaultMotionState.create( groundTransform );
    var rbInfo = Bump.RigidBody.RigidBodyConstructionInfo.create( 0, myMotionState, groundShape, Bump.Vector3.create() );
    // var rbInfo = Bump.RigidBody.RigidBodyConstructionInfo.create( 0, myMotionState, groundBoxShape, Bump.Vector3.create() );
    groundBody = Bump.RigidBody.create( rbInfo );

    groundBody.setCollisionFlags( groundBody.getCollisionFlags() | Bump.CollisionObject.CollisionFlags.CF_KINEMATIC_OBJECT );
    groundBody.setActivationState( Bump.CollisionObject.DISABLE_DEACTIVATION );

    dynamicsWorld.addRigidBody( groundBody );

    var _groundBody = Bump.RigidBody.create( rbInfo );

    _groundBody.setCollisionFlags( groundBody.getCollisionFlags() | Bump.CollisionObject.CollisionFlags.CF_KINEMATIC_OBJECT );
    _groundBody.setActivationState( Bump.CollisionObject.DISABLE_DEACTIVATION );

    effectsWorld.addRigidBody( _groundBody );
  }( 20 ));

  var boxCubeShape = Bump.BoxShape.create( Bump.Vector3.create( 0.5, 0.5, 0.5 ) );
  // var sphereShape = Bump.SphereShape.create( Math.sqrt( 1.5*1.5 + 1.5*1.5 + 1.5*1.5 ) );
  var sphereShape = Bump.BoxShape.create( Bump.Vector3.create( 1.5, 1.5, 1.5 ) );
  collisionShapes.push( boxCubeShape );
  collisionShapes.push( sphereShape );

  var createBoxGeometry = function( aryConfig, geometry ) {
    geometry = geometry || new THREE.Geometry();

    for ( var i = aryConfig.length - 1; i >= 0; --i ) {
      if ( aryConfig[ i ] ) {
        var submesh = new THREE.Mesh( new THREE.CubeGeometry( 1, 1, 1, 1, 1, 1, renderer.faceMaterials ), renderer.boxMaterial );
        submesh.position.set(
          ( i % 3 ) - 1,
          Math.floor( i / 3 ) % 3 - 1,
          Math.floor( i / 9 ) % 3 - 1 );
        submesh.updateMatrix();
        THREE.GeometryUtils.merge( geometry, submesh );
      }
    }

    return geometry;
  };

  var createBoxMesh = function( aryConfig ) {
    var mesh = new THREE.Mesh( createBoxGeometry( aryConfig ), renderer.boxMaterial );
    mesh.useQuaternion = true;
    mesh.config = aryConfig;

    return mesh;
  };

  var createBoxShape = function( config, shape ) {
    shape = shape || Bump.CompoundShape.create();
    var childTransform = Bump.Transform.create();

    for ( var i = config.length - 1; i >= 0; --i ) {
      if ( config[ i ] ) {
        childTransform.setIdentity();
        childTransform.getOrigin().setValue(
          ( i % 3 ) - 1,
          Math.floor( i / 3 ) % 3 - 1,
          Math.floor( i / 9 ) % 3 - 1 );
        shape.addChildShape( childTransform, boxCubeShape );
      }
    }

    return shape;
  };

  var createCompoundCube = function( config, motionState ) {
    var shape = Bump.CompoundShape.create();
    var childTransform = Bump.Transform.create();

    for ( var i = config.length - 1; i >= 0; --i ) {
      if ( config[ i ] ) {
        childTransform.setIdentity();
        childTransform.getOrigin().setValue(
          ( i % 3 ) - 1,
          Math.floor( i / 3 ) % 3 - 1,
          Math.floor( i / 9 ) % 3 - 1 );
        shape.addChildShape( childTransform, boxCubeShape );
      }
    }

    // shape.calculateLocalInertia( 1, localInertia );

    var rbInfo = Bump.RigidBody.RigidBodyConstructionInfo.create( 0, motionState, shape, Bump.Vector3.create() );

    var body = Bump.RigidBody.create( rbInfo );

    body.setCollisionFlags( body.getCollisionFlags() | Bump.CollisionObject.CollisionFlags.CF_KINEMATIC_OBJECT );
    body.setActivationState( Bump.CollisionObject.DISABLE_DEACTIVATION );

    return body;
  };

  var createCube = function( i, j, k, startRotation, config ) {
    if ( !startRotation ) {
      startRotation = Bump.Quaternion.getIdentity();
    }

    var localInertia = Bump.Vector3.create();
    // boxCubeShape.calculateLocalInertia( 1, localInertia );

    // sphereShape = boxCubeShape;
    sphereShape.calculateLocalInertia( 1, localInertia );

    // var shape = Math.random() > 0.5 ? sphereShape : boxCubeShape;
    shape = config ? sphereShape : boxCubeShape;

    shape.calculateLocalInertia( 1, localInertia );

    var startTransform = Bump.Transform.create();
    startTransform.setIdentity();
    startTransform.setRotation( startRotation );
    startTransform.setOrigin( Bump.Vector3.create( i, j, k ) );

    var myMotionState = Bump.DefaultMotionState.create( startTransform );
    var rbInfo = Bump.RigidBody.RigidBodyConstructionInfo.create( 1, myMotionState, shape, localInertia );
    var body = Bump.RigidBody.create( rbInfo );

    body.setFriction( 0.1 );

    var world = config ? dynamicsWorld : effectsWorld;
    // var world = dynamicsWorld; 

    world.addRigidBody( body );

    if ( config ) {

      effectsWorld.addRigidBody( createCompoundCube( config, myMotionState ) );

    } else {
      return body;
    }
  };

  function removeOne( config, compoundShapeRef, geometryRef ) {
    var i = -1;
    for ( i = config.length - 1; i >= 0; i-- ) {
      if ( config[ i ] ) {
        config[ i ] = 0;
        break;
      }
    }

    if ( i !== -1 ) {
      // var x = ( i % 3 ) - 1,
      //     y = Math.floor( i / 3 ) % 3 - 1,
      //     z = Math.floor( i / 9 ) % 3 - 1;
      // 
      // for ( var index = 0; index < compoundShape.getNumChildShapes(); ++index ) {
      //   var transform = compoundShape.getChildTransform( index ),
      //       origin = transform.getOrigin();
      // 
      //   if ( origin.x === x && origin.y === y && origin.z === z ) {
      //     compoundShape.removeChildShapeByIndex( index );
      //     break;
      //   }
      // }
      if ( compoundShapeRef ) {
        createBoxShape( config, compoundShapeRef );
      }

      if ( geometryRef ) {
        createBoxGeometry( config, geometryRef );
      }
    }

    return i;
  }

  var renderer = new THREEWrapper();
  renderer.init();

  renderer.addBox({ size: 20, wireframe: true });
  renderer.renderer.sortObjects = true;

  (function() {
    var num = 3;
    var j = 4;
    for ( var i = 0; i < num; ++i ) {
      for ( var k = 0; k < num; ++k ) {
        var config = [
          1, 1, 1, 1, 1, 1, 1, 1, 1,
          1, 1, 1, 1, 1, 1, 1, 1, 1,
          1, 1, 1, 1, 1, 1, 1, 1, 1 ];

        createCube( j, i - (num - 1) / 2, k - (num - 1) / 2, undefined, config );

        // var mesh = renderer.addBox({ size: 1 });
        // mesh.matrixAutoUpdate = false;
        // mesh.matrixWorldAutoUpdate = false;
        var mesh = createBoxMesh( config );
        renderer.scene.add( mesh );
        renderer.meshes.push( mesh );
        // mesh.add( new THREE.Mesh( new THREE.SphereGeometry( sphereShape.getRadius() ), new THREE.MeshBasicMaterial( { color: Math.random() * 0xffffff, opacity: 0.2, transparent: true, wireframe: true } ) ) );
        // for ( var l = -1; l < 2; ++l ) { for ( var m = -1; m < 2; ++m ) { for ( var n = -1; n < 2; ++n ) {
        //   if ( l === 0 && m === 0 && n === 0 ) { continue; }
        //   var submesh = new THREE.Mesh( new THREE.CubeGeometry( 1, 1, 1, 1, 1, 1, renderer.faceMaterials ), renderer.boxMaterial );
        //   submesh.position.set( l, m, n );
        //   submesh.updateMatrix();
        //   THREE.GeometryUtils.merge( mesh.geometry, submesh );
        // }
        // }
        // }

      }
    }
  }());

  var groundRot = Bump.Quaternion.createWithEuler( 0, 0, Math.PI * 0.003 );
  var quat = Bump.Quaternion.create();
  var newTransform = Bump.Transform.create();
  var tmpMat = Bump.Matrix3x3.create();

  var rate = Math.PI / 60 / 5;
  var amp  = rate / 2;

  var ReadOut = function() {
    this.numManifolds = 0;
    this.numContacts = 0;
    this.bigContacts = 0;

    this.effectsHz = 30;
    this.dynamicHz = 30;
    this.dynamicStep = 30;
  };
  var readOut = new ReadOut();
  var gui = new dat.GUI();
  gui.add( readOut, 'numManifolds' ).listen();
  gui.add( readOut, 'numContacts' ).listen();
  gui.add( readOut, 'bigContacts' ).listen();
  // gui.add( readOut, 'effectsHz' );
  // gui.add( readOut, 'dynamicHz' );
  // gui.add( readOut, 'dynamicStep' );

  var startSimulation = function() {
    var time = 0;
    var effectTime = 0;

    var step = function () {
      time += 16;

      groundRot.setEuler( 0, 0, rate + amp * Math.sin( time / 500 ) );
      groundBody.getMotionState().getWorldTransform( newTransform );
      newTransform.basis.multiplyMatrix( tmpMat.setRotation( groundRot ), newTransform.basis );
      groundBody.getMotionState().setWorldTransform( newTransform );

      stats.begin();
      // if ( dynamicsWorld.getNumCollisionObjects() < effectsWorld.getNumCollisionObjects() && effectsDispatcher.getNumManifolds() === 0 )
      if ( time > effectTime + 1 / readOut.effectsHz * 1000 ) {
        effectTime += 1 / readOut.effectsHz * 1000;
        effectsWorld.stepSimulation( 1 / readOut.effectsHz, 1, 1 / readOut.effectsHz );
      }
      dynamicsWorld.stepSimulation( 1 / readOut.dynamicStep, 20, 1 / readOut.dynamicHz );
      stats.end();

      readOut.numManifolds = dispatcher.getNumManifolds();
      readOut.numContacts = 0;
      // readOut.bigContacts = 0;
      for ( var man = readOut.numManifolds - 1; man >= 0; --man ) {
        var manifold = dispatcher.getManifoldByIndexInternal( man );

        var rbA = Bump.RigidBody.upcast( manifold.getBody0() ),
            rbB = Bump.RigidBody.upcast( manifold.getBody1() ),
            indexA = dynamicsWorld.getCollisionObjectArray().indexOf( rbA ),
            indexB = dynamicsWorld.getCollisionObjectArray().indexOf( rbB ),
            meshA = indexA === -1 ? null : renderer.meshes[ indexA ],
            meshB = indexB === -1 ? null : renderer.meshes[ indexB ],
            effectsRbA = effectsWorld.getCollisionObjectArray()[ indexA ],
            effectsRbB = effectsWorld.getCollisionObjectArray()[ indexB ],
            configAChanged = false,
            configBChanged = false;

        var numContacts = manifold.getNumContacts();
        readOut.numContacts += numContacts;
        for ( var con = numContacts - 1; con >= 0; --con ) {
          var pt = manifold.getContactPoint( con );
          if ( pt.getDistance() < 0 ) {
            if ( pt.appliedImpulse > 2 && readOut.bigContacts < 25 ) {
              readOut.bigContacts ++;

              if ( meshA && meshA.config ) {
                configAChanged = true;
                var pieceIndex = removeOne( meshA.config );

                var pos = Bump.Vector3.create(
                  ( pieceIndex % 3 ) - 1,
                  Math.floor( pieceIndex / 3 ) % 3 - 1,
                  Math.floor( pieceIndex / 9 ) % 3 - 1 );
                // console.log( pos.close() );
                rbA.getWorldTransform().transform( pos, pos );
                var cube = createCube( pos.x, pos.y, pos.z );
                cube.getWorldTransform().getBasis().assign( rbA.getWorldTransform().getBasis() );
                // console.log( pos, cube.getWorldTransform().getOrigin().clone() );
                renderer.addBox({ size: 1 });
              }
              if ( meshB && meshB.config ) {
                configBChanged = true;
                var pieceIndex = removeOne( meshB.config );

                var pos = Bump.Vector3.create(
                  ( pieceIndex % 3 ) - 1,
                  Math.floor( pieceIndex / 3 ) % 3 - 1,
                  Math.floor( pieceIndex / 9 ) % 3 - 1 );
                rbB.getWorldTransform().transform( pos, pos );
                var cube = createCube( pos.x, pos.y, pos.z );
                cube.getWorldTransform().getBasis().assign( rbB.getWorldTransform().getBasis() );
                renderer.addBox({ size: 1 });
              }
            }
          }
        }

        if ( configAChanged ) {
          var geomA = new THREE.Geometry(),
              newMeshA = new THREE.Mesh( geomA, renderer.boxMaterial );

          createBoxGeometry( meshA.config, geomA );
          newMeshA.useQuaternion = true;
          newMeshA.config = meshA.config;

          renderer.meshes[ indexA ] = newMeshA;
          renderer.scene.remove( meshA );
          renderer.scene.add( newMeshA );

          effectsWorld.removeRigidBody( effectsRbA );
          effectsWorld.addRigidBody( createCompoundCube( meshA.config, rbA.getMotionState() ) );
        }
        if ( configBChanged ) {
          var geomB = new THREE.Geometry(),
              newMeshB = new THREE.Mesh( geomB, renderer.boxMaterial );

          createBoxGeometry( meshB.config, geomB );
          newMeshB.useQuaternion = true;
          newMeshB.config = meshB.config;

          renderer.meshes[ indexB ] = newMeshB;
          renderer.scene.remove( meshB );
          renderer.scene.add( newMeshB );

          effectsWorld.removeRigidBody( effectsRbB );
          effectsWorld.addRigidBody( createCompoundCube( meshB.config, rbB.getMotionState() ) );
        }
      }

      for ( var i = 0; i < dynamicsWorld.getNumCollisionObjects(); ++i ) {
        var colObj = dynamicsWorld.getCollisionObjectArray()[i];
        var body = Bump.RigidBody.upcast( colObj );
        body.getMotionState().getWorldTransform( newTransform );

        var mesh = renderer.meshes[ i ];
        mesh.position.copy( newTransform.origin );
        mesh.quaternion.copy( newTransform.getRotation( quat ) );
        mesh.updateMatrix();
      }
      for ( ; i < effectsWorld.getNumCollisionObjects(); ++i ) {
        var colObj = effectsWorld.getCollisionObjectArray()[ i ];
        var body = Bump.RigidBody.upcast( colObj );
        body.getMotionState().getWorldTransform( newTransform );

        var mesh = renderer.meshes[ i ];
        mesh.position.copy( newTransform.origin );
        mesh.quaternion.copy( newTransform.getRotation( quat ) );
        mesh.updateMatrix();
      }

      renderer.render();

      setTimeout( step, 0 );
    };

    window.requestAnimationFrame( step );
  };

  var keylistener = function( evt ) {
    // Enter key
    if ( evt.keyCode === 13 ) {
      document.body.removeEventListener( 'keyup', keylistener );

      startSimulation();
    }
  };

  document.body.addEventListener( 'keyup', keylistener );

}());
