/*global Stats:false THREEWrapper:false*/

(function() {
  var stats = new Stats();
  // stats.setMode(1);

  // Align top-left
  stats.domElement.style.position = 'absolute';
  stats.domElement.style.left = '0px';
  stats.domElement.style.top = '0px';

  document.body.appendChild( stats.domElement );

  function Options() {
    this.shape = 'box';
    this.hertz = 30;
    this.numShapesWidth = 4;
    this.numShapesDepth = 4;
    this.useMargin = true;
    this.disallowDeactivation = true;
    this.start = function() {
      keylistener({ keyCode: 13 });
      this.start = function() {};
      if ( this.startController ) {
        this.startController.remove();
      }
    };
  };
  var options = new Options();
  var gui = new dat.GUI();
  gui.add( options, 'disallowDeactivation' );
  gui.add( options, 'hertz' );
  gui.add( options, 'shape', [ 'box', 'hull-box', 'hull-diamond', 'hull-pyramid', 'hull-hemisphere' ] ).onChange( function( value ) {
    switch ( value ) {
      case 'hull-box':
        boxCubeShape = Bump.ConvexHullShape.create( cubePoints );
        break;
      case 'hull-diamond':
        boxCubeShape = Bump.ConvexHullShape.create( diamondPoints );
        break;
      case 'hull-pyramid':
        boxCubeShape = Bump.ConvexHullShape.create( pyramidPoints );
        break;
      case 'hull-hemisphere':
        boxCubeShape = Bump.ConvexHullShape.create( hsPoints );
        break;
      default:
        boxCubeShape = Bump.BoxShape.create( Bump.Vector3.create( 0.5, 0.5, 0.5 ) );
        break;
    }
  } );
  gui.add( options, 'numShapesWidth' );
  gui.add( options, 'numShapesDepth' );
  options.startController = gui.add( options, 'start' );

  var collisionConfiguration = Bump.DefaultCollisionConfiguration.create();
  var dispatcher = Bump.CollisionDispatcher.create( collisionConfiguration );
  var overlappingPairCache = Bump.DbvtBroadphase.create();
  var solver = Bump.SequentialImpulseConstraintSolver.create();
  var dynamicsWorld = Bump.DiscreteDynamicsWorld.create( dispatcher, overlappingPairCache, solver, collisionConfiguration );
  dynamicsWorld.setGravity( Bump.Vector3.create( 0, -20, 0 ) );

  var collisionShapes = [];

  var groundBody;

  var boxCubeShape = Bump.BoxShape.create( Bump.Vector3.create( 0.5, 0.5, 0.5 ) );
  var cubePoints = [
    0.5, 0.5, 0.5,
    -0.5, 0.5, 0.5,
    -0.5, 0.5, -0.5,
    0.5, 0.5, -0.5,

    0.5, -0.5, 0.5,
    -0.5, -0.5, 0.5,
    -0.5, -0.5, -0.5,
    0.5, -0.5, -0.5
  ];
  var diamondPoints = [
    0, 0.5, 0,

    0.5, 0, 0.5,
    0.5, 0, -0.5,
    -0.5, 0, 0.5,
    -0.5, 0, -0.5,

    0, -0.5, 0
  ];

  var pyramidPoints = [
    0, 0.5, 0,

    0.5, 0, 0.5,
    0.5, 0, -0.5,
    -0.5, 0, 0.5,
    -0.5, 0, -0.5
  ];
  // (function() {
  //   var newlist = [];
  //   while ( cubePoints.length ) {
  //     var rnd = Math.floor( cubePoints.length / 3 * Math.random() ) * 3
  //     newlist.push( cubePoints[ rnd + 0 ] );
  //     newlist.push( cubePoints[ rnd + 1 ] );
  //     newlist.push( cubePoints[ rnd + 2 ] );
  //     cubePoints.splice( rnd, 3 );
  //   }
  //   cubePoints = newlist;
  //   console.log( cubePoints );
  // })();
  cubePoints = cubePoints.map( function( v ) {
    return ( v > 0 ? v - Bump.CONVEX_DISTANCE_MARGIN : v + Bump.CONVEX_DISTANCE_MARGIN );
  } );
  var hsPoints = [
    0, -0.5, 0.5,
    -0.353, -0.5, 0.353,
    -0.5, -0.5, 0,
    -0.353, -0.5, -0.353,
    0, -0.5, -0.5,
    0.353, -0.5, -0.353,
    0.5, -0.5, 0,
    0.353, -0.5, 0.353,

    0, -0.25, 0.447,
    -0.316, -0.25, 0.316,
    -0.447, -0.25, 0,
    -0.316, -0.25, -0.315,
    0, -0.25, -0.447,
    0.316, -0.25, -0.316,
    0.447, -0.25, 0,
    0.316, -0.25, 0.316,

    0, 0, 0
  ];
  // var boxCubeShape = Bump.ConvexHullShape.create( cubePoints );
  // var boxCubeShape = Bump.SphereShape.create( 0.5 );
  collisionShapes.push( boxCubeShape );

  var createCube = function( i, j, k, startRotation ) {
    if ( !startRotation ) {
      startRotation = Bump.Quaternion.getIdentity();
    }

    var localInertia = Bump.Vector3.create();
    boxCubeShape.calculateLocalInertia( 1, localInertia );

    var startTransform = Bump.Transform.create();
    startTransform.setIdentity();
    startTransform.setRotation( startRotation );
    startTransform.setOrigin( Bump.Vector3.create( i, j, k ) );

    var myMotionState = Bump.DefaultMotionState.create( startTransform );
    var rbInfo = Bump.RigidBody.RigidBodyConstructionInfo.create( 1, myMotionState, boxCubeShape, localInertia );
    var body = Bump.RigidBody.create( rbInfo );

    body.setFriction( 0.5 );

    dynamicsWorld.addRigidBody( body );
  };

  var renderer = new THREEWrapper();
  renderer.init();

  renderer.addBox({ size: 20, wireframe: true });

  var groundRot = Bump.Quaternion.createWithEuler( 0, 0, Math.PI * 0.003 );
  var quat = Bump.Quaternion.create();
  var newTransform = Bump.Transform.create();
  var tmpMat = Bump.Matrix3x3.create();

  var rate = Math.PI / 60 / 5;
  var amp  = rate / 2;

  var startSimulation = function() {
    var time = 0;

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
      if ( options.disallowDeactivation ) {
        groundBody.setActivationState( Bump.CollisionObject.DISABLE_DEACTIVATION );
      }

      dynamicsWorld.addRigidBody( groundBody );
    }( 20 ));

    (function() {
      var width = options.numShapesWidth;
      var height = options.numShapesDepth;
      var j = 4;
      for ( var i = 0; i < width; ++i ) {
        for ( var k = 0; k < height; ++k ) {
          createCube( i - (width - 1) / 2, j, k - (height - 1) / 2 );

          renderer.addBox({ size: 1 });

        }
      }
    }());

    var step = function () {
      time += 16;

      // groundRot.setEuler( 0, 0, rate + amp * Math.sin( time / 500 ) );
      // groundBody.getMotionState().getWorldTransform( newTransform );
      // newTransform.basis.multiplyMatrix( tmpMat.setRotation( groundRot ), newTransform.basis );
      // groundBody.getMotionState().setWorldTransform( newTransform );

      stats.begin();
      dynamicsWorld.stepSimulation( 1 / 6, 20, 1 / options.hertz );
      stats.end();

      for ( var i = 0; i < dynamicsWorld.getNumCollisionObjects(); ++i ) {
        var colObj = dynamicsWorld.getCollisionObjectArray()[i];
        var body = Bump.RigidBody.upcast( colObj );
        body.getMotionState().getWorldTransform( newTransform );

        var mesh = renderer.meshes[ i ];
        mesh.position.copy( newTransform.origin );
        mesh.quaternion.copy( newTransform.getRotation( quat ) );
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
