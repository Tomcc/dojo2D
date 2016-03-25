#pragma once

#include "common_header.h"

namespace Phys {
	class Material;
	class Body;

	class BodyPart {
	public:
		friend class Body;

		const Material& material;
		Body& body;

		BodyPart(Body& body, const Material& material);

		b2Fixture& getFixture() const;

		float getMinimumDistanceTo(const Vector& pos) const;

		//HACK is there a way to give a shared ptr to the collision system without enable_shared_from_this?
		std::weak_ptr<BodyPart> _getWeakPtr() {
			return mSelfWeakPtr;
		}

	protected:
		b2Fixture* fixture = nullptr;
		std::weak_ptr<BodyPart> mSelfWeakPtr;

		//use this to notify the bodypart the shared ptr it's stored in
		void _notifySharedPtr(Shared<BodyPart>& me) {
			mSelfWeakPtr = me;
		}
	};
}
