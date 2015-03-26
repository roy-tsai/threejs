function fk (theta,joints)
{
	var m = new THREE.Matrix4();
	var localzero = new THREE.Vector3(0,0,0);
	joints[0] = new THREE.Vector3(0,0,0);


	m.makeRotationY (theta[0]);
	m.multiply (new THREE.Matrix4().makeTranslation(50,0,0));
	localzero.applyMatrix4(m);
	joints[1].copy (localzero);	
	
	m.multiply (new THREE.Matrix4().makeRotationY (theta[1]));
	m.multiply (new THREE.Matrix4().makeTranslation(75,0,0));

	localzero = new THREE.Vector3(0,0,0);
	localzero.applyMatrix4(m);
	joints[2].copy (localzero);


}

function CLAMP(x,lo,hi) { 
	return x > hi ? hi : ( x < lo ? lo : x );
	
}


function ik_ccd (target, theta){
	var base = new THREE.Vector3();	
	var Tmpt = new THREE.Vector3();
	var dx = new THREE.Vector3();			
	var t_target = new THREE.Vector3();
	var t_reach = new THREE.Vector3();

	var joints = [];
	for (var i = 0; i < 3; i++) joints[i] = new THREE.Vector3();
	
	fk(theta,joints);	

	
	Cend = joints[2];

	var eps = 1e-4;



	for (var iter = 0; iter < 10; iter++) {			
		for (var i = 1; i >= 0; i--) {  // distal joint first

			base = joints[i];
	
			Tmpt.subVectors (target, base);
			t_target.copy(Tmpt.normalize());
			Tmpt.subVectors (Cend, base);
			t_reach.copy(Tmpt.normalize());

			Tmpt.crossVectors(t_reach,t_target);

			var dotV = t_reach.dot(t_target);
			var angle = Math.acos (CLAMP(dotV, -1,1));
			var sign = Tmpt.dot(new THREE.Vector3(0,1,0)) > 0 ? 1.0 : -1.0;
			theta[i] += sign*angle;

			// joint limit
			//theta[1] = CLAMP (theta[1], -3,3);
			//theta[0] = CLAMP (theta[0], 0, 3);
			
			if(i == 1)
				theta2 = theta[i];
			else
				theta1 = theta[i];

			fk(theta,joints);
			
			Cend = joints[2];
						
			
			dx.subVectors(target,Cend);
			//console.log ("err " + dx.length());
			if (dx.length() < eps) {
				return 1;
				
			}
		}
	
	} 

	if (iter < 10) 
		return 1;
	else {
		console.log ("restore (do not converge)\n");
		return 0;
	}	

}
