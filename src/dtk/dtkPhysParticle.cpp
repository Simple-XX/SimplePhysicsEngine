
/**
 * @file dtkPhysParticle.cpp
 * @brief dtkPhysParticle 实现
 * @author Zone.N (Zone.Niuzh@hotmail.com)
 * @version 1.0
 * @date 2023-10-31
 * @copyright MIT LICENSE
 * https://github.com/Simple-XX/SimplePhysicsEngine
 * @par change log:
 * <table>
 * <tr><th>Date<th>Author<th>Description
 * <tr><td>2023-10-31<td>Zone.N<td>迁移到 doxygen
 * </table>
 */

#include "dtkPhysParticle.h"

namespace dtk
{
	dtkPhysParticle::dtkPhysParticle( const GK::Point3& position, const double& lifetime, const double& mass, const dtkT3<double>& vel )
	{
		mPoint = position;
		mMass = mass;
		mVel = vel;
		mLifetime = lifetime;

		mActive = true;

		mResistCoef = 0.0;
	}

	dtkPhysParticle::~dtkPhysParticle()
	{

	}

	bool dtkPhysParticle::Update( double timeslice )
	{
		if(!mActive)
		{
			mForceAccum = dtkT3<double>(0,0,0);
			return true;
		}

		// compute acceleration
		mAccel = ( mForceAccum - mVel * mResistCoef ) / mMass;
		mForceAccum = dtkT3<double>( 0, 0, 0 );

		// get point
		dtkT3<double> p;
		p = GetPosition();

		// Euler step
		p = p + mVel * timeslice;		
		mVel = mVel + mAccel * timeslice;

		// set point
		mPoint = GK::Point3(p.x, p.y, p.z);

		mLifetime -= timeslice;
		if( mLifetime < 0 )
			SetActive(false);

		return true;
	}
}
