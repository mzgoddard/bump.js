module( 'SphereBoxCollisionAlgorithm.getSphereDistance' );

function truncate( v, precision ) {
  v.x = Math.ceil( v.x * precision ) / precision;
  v.y = Math.ceil( v.y * precision ) / precision;
  v.z = Math.ceil( v.z * precision ) / precision;
  return v;
}

test( 'sphere-box x axis', function() {

  expect( 10 );

  var manifold = Bump.PersistentManifold.create();
  var caci = Bump.CollisionAlgorithmConstructionInfo.create();

  var sphereShape = Bump.SphereShape.create( 0.5 );
  var boxShape = Bump.BoxShape.create( Bump.Vector3.create( 0.5, 0.5, 0.5 ) );

  var localInertia = Bump.Vector3.create();
  sphereShape.calculateLocalInertia( 1, localInertia );

  var startTransform = Bump.Transform.create();
  startTransform.setIdentity();
  startTransform.setOrigin( Bump.Vector3.create( 0, 0, 0 ) );

  var myMotionState = Bump.DefaultMotionState.create( startTransform );
  var rbInfo = Bump.RigidBody.RigidBodyConstructionInfo.create(
    1, myMotionState, sphereShape, localInertia );

  var sphereBody = Bump.RigidBody.create( rbInfo );

  startTransform = Bump.Transform.create();
  startTransform.setIdentity();
  startTransform.setOrigin( Bump.Vector3.create( 0.9, 0, 0 ) );

  myMotionState = Bump.DefaultMotionState.create( startTransform );
  rbInfo = Bump.RigidBody.RigidBodyConstructionInfo.create(
    1, myMotionState, boxShape, localInertia );

  var boxBody = Bump.RigidBody.create( rbInfo );

  var sbca = Bump.SphereBoxCollisionAlgorithm.create(
    manifold, caci, sphereBody, boxBody );

  ok( sbca, 'SphereBoxCollisionAlgorithm created.' );

  var pOnBox = Bump.Vector3.create(),
      pOnSphere = Bump.Vector3.create(),
      sphereCenter = Bump.Vector3.create( 0, 0, 0 ),
      dist;

  console.log( sphereBody.getWorldTransform().getOrigin() );
  // console.log( sbca.getSphereDistance( boxBody, pOnBox, pOnSphere, sphereCenter, sphereShape.getRadius() ), pOnBox, pOnSphere );

  dist = sbca.getSphereDistance( boxBody, pOnBox, pOnSphere, sphereCenter, sphereShape.getRadius() );

  ok( dist < Bump.SIMD_EPSILON, 'sphereDistance less than EPSILON.' );
  deepEqual( sphereCenter, Bump.Vector3.create( 0, 0, 0 ), 'sphereCenter unchanged.' );
  deepEqual( boxShape.getHalfExtentsWithMargin(), Bump.Vector3.create( 0.5, 0.5, 0.5 ) );
  deepEqual( boxBody.getWorldTransform().getOrigin(), Bump.Vector3.create( 0.9, 0, 0 ) );
  deepEqual( pOnSphere, Bump.Vector3.create( 0.5, 0, 0 ), 'point on sphere is correct.' );
  deepEqual( pOnBox, Bump.Vector3.create( 0.9 - boxShape.getHalfExtentsWithoutMargin().x, 0, 0 ), 'point on box is correct.' );

  boxBody.getWorldTransform().setOrigin( Bump.Vector3.create( 0.1, 0, 0 ) );
  dist = sbca.getSphereDistance( boxBody, pOnBox, pOnSphere, sphereCenter, sphereShape.getRadius() );

  ok( dist < Bump.SIMD_EPSILON, 'sphereDistance less than EPSILON. (' + dist + ')' );
  deepEqual( pOnSphere, Bump.Vector3.create( 0.5, 0, 0 ), 'point on sphere is correct.' );

  deepEqual(
    truncate( pOnBox, 1000 ),
    Bump.Vector3.create(
      0.1 -
        boxShape.getHalfExtentsWithoutMargin().x +
        boxShape.getMargin(),
      0,
      0 ),
    'point on box is correct.' );

  console.log( dist, pOnBox, pOnSphere );

});
