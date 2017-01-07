/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.ConvexHullShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.demos.opengl.DemoApplication;
import com.bulletphysics.demos.opengl.GLDebugDrawer;
import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.demos.opengl.LWJGL;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import javax.vecmath.Vector3f;

import org.lwjgl.LWJGLException;

import static com.bulletphysics.demos.opengl.IGL.*;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;

/**
 * BasicDemo is good starting point for learning the code base and porting.
 * 
 * @author jezek2
 */
public class SkittlesSimulation extends DemoApplication implements Runnable {

	public Map<String,Float> mProps;
	RigidBody mBall;
	RigidBody[] mPins;
	Vector3f mInitialVelocity;
	private boolean mPaused;
	private float mX;
	private static NumberFormat NF=new DecimalFormat("0.000");
	private float mTimeMultipler;
	
	// keep the collision shapes, for deletion/cleanup
	private ObjectArrayList<CollisionShape> collisionShapes = new ObjectArrayList<CollisionShape>();
	private BroadphaseInterface broadphase;
	private CollisionDispatcher dispatcher;
	private ConstraintSolver solver;
	private DefaultCollisionConfiguration collisionConfiguration;

	public SkittlesSimulation(IGL gl) {
		super(gl);
	}
	
	@Override
	public void clientMoveAndDisplay() {
		if (mPaused) return;
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		// simple dynamics world doesn't handle fixed-time-stepping
		float ms = getDeltaTimeMicroseconds();

		// step the simulation
		if (dynamicsWorld != null) {
			dynamicsWorld.stepSimulation( mTimeMultipler * ms / 1000000f );
			dynamicsWorld.debugDrawWorld();
			
		}

		renderme();
	}

	@Override
	public void displayCallback() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		renderme();
		if (dynamicsWorld != null) {
			dynamicsWorld.debugDrawWorld();
		}
	}
	
	static Random sRandom=new Random();

	public void initPhysics() {
		
		
		Map<String,Float> aProps=mProps;
		float alleyLength=aProps.get("alleyLength");
		float diamond=aProps.get("diamond");
		float ballDiameter=aProps.get("ballDiameter");
		float ballWeight=aProps.get("ballWeight");
		float pinHeight=aProps.get("pinHeight");
		float pinDiameter=aProps.get("pinDiameter");
		float pinWeight=aProps.get("pinWeight");
		float restitution=aProps.get("restitution");
		float friction=aProps.get("friction");
		
		float throwPosition=aProps.get("throwPosition");
		float throwAim=aProps.get("throwAim");
		float throwVelocity=aProps.get("throwVelocity");
		

		float zOffset=-alleyLength*0.5f;

		// collision configuration contains default setup for memory, collision setup
		collisionConfiguration = new DefaultCollisionConfiguration();

		// use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		dispatcher = new CollisionDispatcher(collisionConfiguration);

		broadphase = new DbvtBroadphase();

		// the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
		SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
		sol.setRandSeed(sRandom.nextLong());
		solver = sol;
		
		dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
		dynamicsWorld.getSolverInfo().numIterations=10;

		dynamicsWorld.setGravity(new Vector3f(0f, -10f, 0f));

		// create a few basic rigid bodies
		CollisionShape groundShape = new BoxShape(new Vector3f(diamond*0.75f, 0.25f, alleyLength*0.5f+diamond*0.7f));
		//CollisionShape groundShape = new StaticPlaneShape(new Vector3f(0, 1, 0), 50);

		collisionShapes.add(groundShape);

		Transform groundTransform = new Transform();
		groundTransform.setIdentity();
		groundTransform.origin.set(0, 0, zOffset);

		// We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
		{
			float mass = 0f;

			// rigidbody is dynamic if and only if mass is non zero, otherwise static
			boolean isDynamic = (mass != 0f);

			Vector3f localInertia = new Vector3f(0, 0, 0);
			if (isDynamic) {
				groundShape.calculateLocalInertia(mass, localInertia);
			}

			// using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			DefaultMotionState myMotionState = new DefaultMotionState(groundTransform);
			RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, myMotionState, groundShape, localInertia);
			RigidBody body = new RigidBody(rbInfo);
			body.setFriction(friction);
			body.setRestitution(0.1f);
			// add the body to the dynamics world
			dynamicsWorld.addRigidBody(body);
		}
		
		{
			CollisionShape ball = new SphereShape(ballDiameter*0.5f);
			collisionShapes.add(ball);
			Vector3f localInertia = new Vector3f(0, 0, 0f);
			ball.calculateLocalInertia(ballWeight, localInertia);
			
			Transform startTransform = new Transform();
			startTransform.setIdentity();
			startTransform.origin.set(-throwPosition*diamond*0.5f,0.4f,zOffset-alleyLength*0.5f);
			
			double angle = Math.atan2(alleyLength, (throwPosition-throwAim)*diamond*0.5f);
			mInitialVelocity=new Vector3f((float)(Math.cos(angle)*throwVelocity),0f,(float)(Math.sin(angle)*throwVelocity));
			
			// using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			DefaultMotionState myMotionState = new DefaultMotionState(startTransform);
			RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(ballWeight, myMotionState, ball, localInertia);
			RigidBody body = new RigidBody(rbInfo);
			body.setFriction(friction);
			body.setRestitution(restitution);
			body.setActivationState(RigidBody.DISABLE_DEACTIVATION);
			dynamicsWorld.addRigidBody(body);
			mBall=body;
			
		}
		

		{
			
			// Create Dynamic Objects
			Transform startTransform = new Transform();
			startTransform.setIdentity();
						
			//build skittle shape
			ObjectArrayList<Vector3f> points=new ObjectArrayList<Vector3f>(100);
			float circumferencePoints=21f;
			float lengthPoints=8f;
			float chopRatio=0.5f; // 1.0 no chop - 0.5 lose half
			float radius=pinDiameter*0.25f;
			float height=pinHeight*0.5f;
			
			float hm=1.0f/(float)(Math.cos(Math.PI*((1-chopRatio)*0.5f)));
			for (float h=0; h<lengthPoints; h++)
			{
				float k=h/(lengthPoints-1); //between 0 and 1
				float a=(float)Math.PI*(k*chopRatio+(1-chopRatio)*0.5f);
				float x=(float)Math.sin(a)*radius;
				float y=(float)(Math.cos(a)*height*hm);
				
				for (float c=0; c<circumferencePoints; c++)
				{
					float ck=c/(circumferencePoints-1); //between 0 and 1
					float b=(float)Math.PI*ck*2;
					float z1=(float)Math.sin(b)*x;
					float z2=(float)Math.cos(b)*x;
					
					points.add(new Vector3f(z1,y,z2));
				}
			}
			CollisionShape colShape=new ConvexHullShape(points);

			
			startTransform.setIdentity();

			// rigidbody is dynamic if and only if mass is non zero, otherwise static

			Vector3f localInertia = new Vector3f(0, 0, 0);
			colShape.calculateLocalInertia(pinWeight, localInertia);

			
			
			float[] positions = new float[] {0.0f,1.0f, -0.5f,0.5f, 0.5f,0.5f, -1.0f,0.0f, 0.0f,0.0f, 1.0f,0.0f, -0.5f,-0.5f, 0.5f,-0.5f, 0.0f,-1.0f };
			mPins=new RigidBody[9];
			for (int i=0; i<9; i++)
			{
				float pinX=positions[i*2];
				float pinY=positions[i*2+1];

				startTransform.origin.set(
						pinX*diamond*0.5f + 0,
						0.27f + pinHeight*0.5f,
						zOffset + pinY*diamond*0.5f + alleyLength*0.5f );
				
				// using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
				DefaultMotionState myMotionState = new DefaultMotionState(startTransform);
				RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(pinWeight, myMotionState, colShape, localInertia);
				RigidBody body = new RigidBody(rbInfo);
				body.setRestitution(restitution);
				body.setFriction(friction);
				body.setDamping(0.0f,0.0f);
				body.setActivationState(RigidBody.DISABLE_DEACTIVATION);
				dynamicsWorld.addRigidBody(body);
				mPins[i]=body;
			}			
		}
			
	}
	
	private void doResult()
	{	Vector3f v=new Vector3f();
		Vector3f p=this.mBall.getCenterOfMassPosition(v);
		Transform t=new Transform();
		int c=0;
		float h=0.25f+mProps.get("pinHeight")*0.5f*0.9f;
		for (int i=0; i<mPins.length; i++)
		{	
			if (mPins[i].getCenterOfMassPosition(v).y<h)
			{	c++;
			}
		}
		//System.out.println(NF.format(mX)+","+(c>0?1:0));
		System.out.println(NF.format(mX)+","+c);
	}
	
	public void setupProcess()
	{	Thread t=new Thread(this);
		t.setDaemon(false);
		t.start();
	}
	
	public void run()
	{
		System.out.println("position,pins");
		Map<String,Float> props=new HashMap();
		
		props.put("alleyLength", 8.9f);
		props.put("diamond", 1.225f);
		props.put("ballDiameter", 0.125f); 
		props.put("ballWeight", 1.4f);
		props.put("pinHeight", 0.25f);
		props.put("pinDiameter", 0.115f);
		props.put("pinWeight", 1.1f);
		props.put("restitution", 0.8f ); //http://hypertextbook.com/facts/2006/restitution.shtml
		props.put("friction", 0.4f ); //http://www.roymech.co.uk/Useful_Tables/Tribology/co_of_frict.htm
		props.put("throwVelocity", 8f);

		float simLeft=-0.21f;
		float xdsimStep=0.000f;
		float simRight=1.25f;
		mTimeMultipler=0.25f; // how much faster
		
		boolean complete=true;
		boolean initialised=false;
		long bowlDuration=0;
		long t0=0;
		try
		{	while(true)
			{	if (complete)
				{
					//throw characteristics
					props.put("throwPosition", 1f);
					props.put("throwAim", simLeft);

					
					mProps=props;
					
					bowlDuration =(long)(5000 / mTimeMultipler);
					if (!initialised)
					{	bowlDuration+=1000L;
					}
					
					initPhysics();
					mBall.setLinearVelocity(mInitialVelocity);
					complete=false;
					mPaused=false;
					mX=simLeft;
					t0=System.currentTimeMillis();
					if (!initialised)
					{	setCameraDistance(5.2f);
						getDynamicsWorld().setDebugDrawer(new GLDebugDrawer(LWJGL.getGL()));
						final DemoApplication demo=this;
						new Thread(new Runnable()
						{	public void run()
							{	try
								{	LWJGL.main(new String[0], 800, 600, "B4069s Skittle-O-Matic", demo);
								} catch (Exception e) {}
							}
						}).start();
						initialised=true;
					}
				}
				else
				{	long now=System.currentTimeMillis();
					if (now-t0>bowlDuration)
					{	doResult();
						mPaused=true;
						try
						{	Thread.sleep(100);
							destroy();
						} catch (Exception e) { e.printStackTrace();}

						complete=true;
						simLeft+=xdsimStep;
						if (simLeft>simRight+0.01f)
						{	break;
						}
					}
				}
				
				Thread.sleep(100);
				
			}
		}
		catch (Throwable t)
		{	t.printStackTrace();
		}
		
	}
	
	public static void main(String[] args) throws LWJGLException {
		IGL lwjgl=LWJGL.getGL();
		SkittlesSimulation ccdDemo = new SkittlesSimulation(lwjgl);
		ccdDemo.setupProcess();
	}
	
}
