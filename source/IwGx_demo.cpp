/*
 * Bullet SDK (2.82) - modified IwGx demo v1.0.0 by @xammond - July'14
 *
 * This is mostly the original "IwGxBasicLighting.mkb" example - added physics
 *
 * THIS MODIFIED CODE IS PROVIDED FOR THE "MARMALADE COMMUNITY CODE" (GITHUB)
 * THE USUAL "AS IS" SITUATION APPLIES (SEE BELOW)
 *
 *
 * >>>To build<<< this demo you must first run "build_bullet282_library.mkb",
 * compiling the project for whichever solution configurations you require.
 * It is recommended from myself; To speed up mesh testing use the release version
 * (unless you are modifying/debugging the library of course!)
 */

/*
 * This file is part of the Marmalade SDK Code Samples.
 *
 * (C) 2001-2012 Marmalade. All Rights Reserved.
 *
 * This source code is intended only as a supplement to the Marmalade SDK.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

/**
 * @page ExampleIwGxBasicLighting IwGx Basic Lighting Example
 *
 * The following example demonstrates how to display a single textured spinning
 * cube, with ambient and diffuse scene lighting. The example allows you to
 * change the lighting values using the debug menu.
 *
 * The main classes used to achieve this are:
 *  - CIwMenu
 *  - CIwMenuItemEditUInt8
 *  - CIwMenuManager
 *  - CIwMenuItemGx
 *  - CIwMaterial
 *  - CIwTexture
 *
 * The main functions used to achieve this are:
 *  - CIwMenu::AddItem()
 *  - IwGxSetLightType()
 *  - IwGxSetLightDirn()
 *  - IwGxLightingOn()
 *  - CIwMenuManager::SetTextCallback();
 *  - CIwMenuManager::SetMainMenuFn()
 *  - CIwMenuManager::Update()
 *
 * The following graphics illustrates the example output.
 *
 * @image html IwGxBasicLightingImage.png
 *
 * @note For more information on IwGx Lighting, see the @ref lighting
 * "Lighting" section.
 *
 * @include IwGxBasicLighting.cpp
 */

#include "IwGx.h"
#include "IwGxPrint.h"
#include "IwMaterial.h"
#include "IwMenu.h"
#include "IwTexture.h"

#include "btBulletDynamicsCommon.h"

//#include "BulletDynamics/Featherstone/..."
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"

// Physics timing.
#define										Hz_physics									60
#define										time_step_physics							(1.0f / Hz_physics)
#define										max_sub_steps_physics						3

// Collision groups.
enum {
	COL_STATIC =			0x0001,
	COL_GENERAL_DYNAMIC =	0x0002,
//	...0x0004 0x0008 0x0010 0x0020...
};

// Fixed timing.
float										s_PhysicsAccumulator;
float										s_DeltaTime;
uint64										s_PrevTime;

// Bullet SDK.
btMultiBodyDynamicsWorld					*s_DynamicsWorld;
btAlignedObjectArray<btCollisionShape*>		s_CollisionShapes;

btBroadphaseInterface						*s_Broadphase;
btDefaultCollisionConfiguration				*s_CollisionConfiguration;
btCollisionDispatcher						*s_Dispatcher;
btMultiBodyConstraintSolver					*s_Solver;		// using multibody solver for future demo expansion

btRigidBody									*s_GroundRigidBody;
btRigidBody									*s_BoxRigidBody;

//
int											g_error;

// Error codes.
enum { no_error, physics_error, };

// Texture object
CIwTexture* s_Texture = NULL;

// Vertex data
const float s = 0x80;
CIwFVec3    s_Verts[24] =
{
    CIwFVec3(-s, -s, -s),
    CIwFVec3( s, -s, -s),
    CIwFVec3( s,  s, -s),
    CIwFVec3(-s,  s, -s),

    CIwFVec3( s, -s, -s),
    CIwFVec3( s, -s,  s),
    CIwFVec3( s,  s,  s),
    CIwFVec3( s,  s, -s),

    CIwFVec3( s, -s,  s),
    CIwFVec3(-s, -s,  s),
    CIwFVec3(-s,  s,  s),
    CIwFVec3( s,  s,  s),

    CIwFVec3(-s, -s,  s),
    CIwFVec3(-s, -s, -s),
    CIwFVec3(-s,  s, -s),
    CIwFVec3(-s,  s,  s),

    CIwFVec3(-s, -s, -s),
    CIwFVec3( s, -s, -s),
    CIwFVec3( s, -s,  s),
    CIwFVec3(-s, -s,  s),

    CIwFVec3( s,  s, -s),
    CIwFVec3(-s,  s, -s),
    CIwFVec3(-s,  s,  s),
    CIwFVec3( s,  s,  s),
};

// Normal data
const float n = 0.577f;
CIwFVec3    s_Norms[24] =
{
    CIwFVec3(-n, -n, -n),
    CIwFVec3( n, -n, -n),
    CIwFVec3( n,  n, -n),
    CIwFVec3(-n,  n, -n),

    CIwFVec3( n, -n, -n),
    CIwFVec3( n, -n,  n),
    CIwFVec3( n,  n,  n),
    CIwFVec3( n,  n, -n),

    CIwFVec3( n, -n,  n),
    CIwFVec3(-n, -n,  n),
    CIwFVec3(-n,  n,  n),
    CIwFVec3( n,  n,  n),

    CIwFVec3(-n, -n,  n),
    CIwFVec3(-n, -n, -n),
    CIwFVec3(-n,  n, -n),
    CIwFVec3(-n,  n,  n),

    CIwFVec3(-n, -n, -n),
    CIwFVec3( n, -n, -n),
    CIwFVec3( n, -n,  n),
    CIwFVec3(-n, -n,  n),

    CIwFVec3( n,  n, -n),
    CIwFVec3(-n,  n, -n),
    CIwFVec3(-n,  n,  n),
    CIwFVec3( n,  n,  n),
};

// UV data
CIwFVec2    s_UVs[24] =
{
    CIwFVec2(0,   0),
    CIwFVec2(1,   0),
    CIwFVec2(1,   1),
    CIwFVec2(0,   1),

    CIwFVec2(0,   0),
    CIwFVec2(1,   0),
    CIwFVec2(1,   1),
    CIwFVec2(0,   1),

    CIwFVec2(0,   0),
    CIwFVec2(1,   0),
    CIwFVec2(1,   1),
    CIwFVec2(0,   1),

    CIwFVec2(0,   0),
    CIwFVec2(1,   0),
    CIwFVec2(1,   1),
    CIwFVec2(0,   1),

    CIwFVec2(0,   0),
    CIwFVec2(1,   0),
    CIwFVec2(1,   1),
    CIwFVec2(0,   1),

    CIwFVec2(0,   0),
    CIwFVec2(1,   0),
    CIwFVec2(1,   1),
    CIwFVec2(0,   1),
};

// Index stream for textured material
uint16      s_QuadList[24] =
{
    0, 3, 2, 1,
    4, 7, 6, 5,
    8, 11, 10, 9,
    12, 15, 14, 13,
    16, 17, 18, 19,
    20, 21, 22, 23,
};

// Local matrix
CIwFMat       s_ModelMatrix;

// Materials
CIwMaterial *s_MaterialBG;
CIwMaterial *s_MaterialCube;

// Ambient scene light colour
CIwColour    s_ColSceneAmb = {0x20, 0x20, 0x20};

// Diffuse scene light colour
CIwColour    s_ColSceneDiff = {0xf0, 0xf0, 0xf0};

//--------------------------------------------------------------------------------
// The following code updates the timer.
//--------------------------------------------------------------------------------
void UpdateTimer()
{
	//
	uint64 TimeNow = s3eTimerGetMs();
	s_DeltaTime = 0.001f * (TimeNow - s_PrevTime);
	s_PrevTime = TimeNow;
}

//--------------------------------------------------------------------------------
// The following code updates the physics. And extracts poses.
//--------------------------------------------------------------------------------
void UpdatePhysics()
{
	//
	s_PhysicsAccumulator += s_DeltaTime;
	// Step through any full time slices.
	while (s_PhysicsAccumulator > 0.0)
	{
		// Update Bullet.
		s_DynamicsWorld->stepSimulation(time_step_physics, max_sub_steps_physics, time_step_physics);

		// Step.
		s_PhysicsAccumulator -= time_step_physics;
	}

	// Extract dynammic actors' matrices. [not needed inside accumulator loop for this demo]
	btTransform t = s_BoxRigidBody->getWorldTransform();
	float m[16];
	t.getOpenGLMatrix(m);
	float *dm = s_ModelMatrix.m[0];
	*dm++ = m[0];		*dm++ = m[1];		*dm++ = m[2];
	*dm++ = m[4];		*dm++ = m[5];		*dm++ = m[6];
	*dm++ = m[8];		*dm++ = m[9];		*dm =   m[10];
	// Scale up to this IwGX demo's coordinate system.
	s_ModelMatrix.SetTrans( -0x100 * CIwFVec3(m[12], m[13], m[14]) );
}

//--------------------------------------------------------------------------------
// The following code initialises the physics (Bullet SDK).
//--------------------------------------------------------------------------------
int InitPhysics()
{
	// Initialise Bullet SDK.
	s_CollisionConfiguration = new btDefaultCollisionConfiguration();
	s_Dispatcher = new btCollisionDispatcher(s_CollisionConfiguration);
	if (!(s_Broadphase = new btDbvtBroadphase())) { /*ShowError("Error creating broadphase (physics)");*/		return physics_error; }
	s_Solver = new btMultiBodyConstraintSolver;
/*	s_solver = new btSequentialImpulseConstraintSolver;*/																// [a NON-multibody alternative solver]
	s_DynamicsWorld = new btMultiBodyDynamicsWorld(s_Dispatcher, s_Broadphase, s_Solver, s_CollisionConfiguration);
/*	s_DynamicsWorld = new btDiscreteDynamicsWorld(s_Dispatcher, s_Broadphase, s_Solver, s_CollisionConfiguration);*/	// [a NON-multibody alternative dynamics world]
	s_DynamicsWorld->setGravity(btVector3(0, -9.81f, 0));

	// Create a ground plane - First create the collision shape.
	btVector3			normal(0, 1, 0);
	btCollisionShape	*groundCollisionShape;
	groundCollisionShape = new btStaticPlaneShape(normal, 0.0f);
	s_CollisionShapes.push_back(groundCollisionShape);
	// Create the groundplane's rigid body. (static)
	btVector3 pos(0.0f, 0.0f, 0.0f);
	btDefaultMotionState *motionState = new btDefaultMotionState( btTransform(btQuaternion(0, 0, 0, 1), pos) );
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(0, motionState, groundCollisionShape, btVector3(0, 0, 0));
	s_GroundRigidBody = new btRigidBody(rigidBodyCI);
	// Add the rigid to the dynamics world, also setting collision flags.
	unsigned short		collideMask(COL_STATIC);
	unsigned short		collidesWith(COL_GENERAL_DYNAMIC /*| COL_SOMETHING_ELSE | ...*/);
	s_DynamicsWorld->addRigidBody(s_GroundRigidBody, collideMask, collidesWith);
	// Configure the rigid with a couple of basics.
	s_GroundRigidBody->setRestitution(0.5f);
	s_GroundRigidBody->setFriction(0.5f);

	// Create a box shape.
	btCollisionShape	*boxCollisionShape;
	boxCollisionShape = new btBoxShape( btVector3(0.5f, 0.5f, 0.5f) );
	s_CollisionShapes.push_back(boxCollisionShape);
	btVector3 inertia(0, 0, 0);
	float mass = 25.0f;
	boxCollisionShape->calculateLocalInertia(mass, inertia);
	// Create the box's rigid body. (dynamic)
	pos = btVector3(0.0f, 6.0f, 0.0f);
	motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), pos));
	rigidBodyCI = btRigidBody::btRigidBodyConstructionInfo( mass, motionState, boxCollisionShape, inertia );
	s_BoxRigidBody = new btRigidBody(rigidBodyCI);
	// Add the rigid to the dynamics world, also setting collision flags.
	collideMask = COL_GENERAL_DYNAMIC;
	collidesWith = COL_STATIC | COL_GENERAL_DYNAMIC /*| COL_SOMETHING_ELSE | ...*/;
	s_DynamicsWorld->addRigidBody(s_BoxRigidBody, collideMask, collidesWith);
	// Configure the rigid with a few basics.
	s_BoxRigidBody->setRestitution(0.5f);
	s_BoxRigidBody->setFriction(0.5f);
	s_BoxRigidBody->setDamping(0.5f, 0.125f);
//	s_BoxRigidBody->setActivationState(DISABLE_DEACTIVATION);		// stay awake

	//
	return no_error;
}

//--------------------------------------------------------------------------------
// The following code de-initialises the physics.
//--------------------------------------------------------------------------------
void DeInitPhysics()
{
	// Rigids.
	btRigidBody *bodies[] = { s_BoxRigidBody, s_GroundRigidBody, };
	enum { numBodies = sizeof(bodies) / sizeof(bodies[0]) };
	//
	for (int i = 0; i < numBodies; ++i)
	{
		btRigidBody *body = bodies[i];
		if (body)
		{
			s_DynamicsWorld->removeRigidBody(body);
//			if (...)
				delete body->getMotionState();
			delete body;
		}
	}

	// Collision shapes.
	for (int i = 0; i < s_CollisionShapes.size(); ++i)
		delete s_CollisionShapes[i];
	s_CollisionShapes.clear();

	// Engine.
	if (s_DynamicsWorld)			{ delete s_DynamicsWorld;				s_DynamicsWorld = NULL; }
	if (s_Solver)					{ delete s_Solver;						s_Solver = NULL; }
	if (s_Dispatcher)				{ delete s_Dispatcher;					s_Dispatcher = NULL; }
	if (s_CollisionConfiguration)	{ delete s_CollisionConfiguration;		s_CollisionConfiguration = NULL; }
	if (s_Broadphase)				{ delete s_Broadphase;					s_Broadphase = NULL; }
}

//--------------------------------------------------------------------------------
// The following code creates a new menu which can be accessed when the example
// is running by pressing F6.
//--------------------------------------------------------------------------------
CIwMenu* DebugCreateMainMenu()
{
    IW_CALLSTACK("DebugCreateMainMenu")

    CIwMenu* pMenu = new CIwMenu;

#ifdef IW_DEBUG
    pMenu->AddItem(new CIwMenuItemGx);
    pMenu->AddItem(new CIwMenuItemEditUInt8("ColSceneAmb.r", &s_ColSceneAmb.r));
    pMenu->AddItem(new CIwMenuItemEditUInt8("ColSceneAmb.g", &s_ColSceneAmb.g));
    pMenu->AddItem(new CIwMenuItemEditUInt8("ColSceneAmb.b", &s_ColSceneAmb.b));
    pMenu->AddItem(new CIwMenuItemEditUInt8("ColSceneDiff.r", &s_ColSceneDiff.r));
    pMenu->AddItem(new CIwMenuItemEditUInt8("ColSceneDiff.g", &s_ColSceneDiff.g));
    pMenu->AddItem(new CIwMenuItemEditUInt8("ColSceneDiff.b", &s_ColSceneDiff.b));
#endif
    return pMenu;
}
//-----------------------------------------------------------------------------
// The following function creates a new material and loads a new texture to be
// used by the spinning cube. It also sets diffuse and ambient lighting for the
// scene using the IwGxSetLightType() function. The IwGxSetLightDirn() is used
// to set the direction of the Diffuse lighting.
//-----------------------------------------------------------------------------
void ExampleInit()
{
	//
	g_error = no_error;
	//
	s_PhysicsAccumulator = 0.0f;
	s_DeltaTime = 0.0f;
	s_Broadphase = NULL;
	s_CollisionConfiguration = NULL;
	s_Dispatcher = NULL;
	s_Solver = NULL;
	s_DynamicsWorld = NULL;
	s_GroundRigidBody = NULL;
	s_BoxRigidBody = NULL;


	// Initialise
    IwGxInit();

//IwMemBucketDebugSetBreakpoint(128);		// <--- revealed Bullet SDK's profiler was abusing buckets! (now disabled via a #define in btQuickprof.h)

	// Initialise physics (Bullet SDK).
	if ((g_error = InitPhysics()))
		{ /*ShowError(g_error);*/ return /*g_error*/;	}

	// Set field of view
    IwGxSetPerspMul(0xa0);

    // Set near and far planes
    IwGxSetFarZNearZ(0x1000, 0x10);

    // Initialise materials
    s_MaterialBG = new CIwMaterial;

    s_MaterialCube = new CIwMaterial;
    s_Texture = new CIwTexture;
    s_Texture->LoadFromFile("./textures/testTexture.bmp");
    s_Texture->Upload();
    s_MaterialCube->SetTexture(s_Texture);

    // Set the view matrix along the -ve z axis, at location (0, 1.5, 1.5) - meters.
    CIwFMat view = CIwFMat::g_Identity;
	view.t.y = 1.5f * -0x100;
	view.t.z = 1.5f * -0x100;
    IwGxSetViewMatrix(&view);

    //-------------------------------------------------------------------------
    // Set up scene lighting
    //-------------------------------------------------------------------------
    // Set single ambient light
    IwGxSetLightType(0, IW_GX_LIGHT_AMBIENT);

    // Set single diffuse light
    IwGxSetLightType(1, IW_GX_LIGHT_DIFFUSE);
    CIwFVec3 dd(0.577f, 0.577f, 0.577f);
    IwGxSetLightDirn(1, &dd);

    // Set up the menu manager
    new CIwMenuManager;
    IwGetMenuManager()->SetTextCallback(IwGxPrintMenuCallback);
    IwGetMenuManager()->SetMainMenuFn(DebugCreateMainMenu);

	// Reset the timer.
	s_PrevTime = s3eTimerGetMs();
}
//-----------------------------------------------------------------------------
// The following function destroys the various classes that have been created
// within the example.
//-----------------------------------------------------------------------------
void ExampleShutDown()
{
	//
	DeInitPhysics();

	delete s_Texture;
	delete IwGetMenuManager();

    delete s_MaterialBG;
    delete s_MaterialCube;

	// Terminate
    IwGxTerminate();
}
//-----------------------------------------------------------------------------
// The following function updates the view and the lighting values.
//-----------------------------------------------------------------------------
bool ExampleUpdate()
{
	// Update physics
	UpdatePhysics();

    // Update menu manager
    IwGetMenuManager()->Update();

    // Update scene light colours from user colours
    // Set single ambient light
    IwGxSetLightCol(0, &s_ColSceneAmb);
    IwGxSetLightCol(1, &s_ColSceneDiff);

	// Update timer
	UpdateTimer();

    return true;
}
//-----------------------------------------------------------------------------
// The following function renders the spinning cube and any debug menu that has
// selected.
//-----------------------------------------------------------------------------
void ExampleRender()
{
    // Clear the screen
    IwGxClear(IW_GX_COLOUR_BUFFER_F | IW_GX_DEPTH_BUFFER_F);

    //-------------------------------------------------------------------------
    // Render cube
    //-------------------------------------------------------------------------
    // Use all lighting
    IwGxLightingOn();

    // Set the model matrix
    IwGxSetModelMatrix(&s_ModelMatrix);

    // Set vertex stream
    IwGxSetVertStreamModelSpace(s_Verts, 24);

    // Set normal stream
    IwGxSetNormStream(s_Norms, 24);

    // Clear colour stream
    IwGxSetColStream(NULL);

    // Set UV stream
    IwGxSetUVStream(s_UVs);

    // Set material
    IwGxSetMaterial(s_MaterialCube);

    // Draw 6 quads
    IwGxDrawPrims(IW_GX_QUAD_LIST, s_QuadList, 24);

    // End drawing
    IwGxFlush();

    // Render menu manager
    IwGetMenuManager()->Render();
    IwGxFlush();

    // Swap buffers
    IwGxSwapBuffers();
}
