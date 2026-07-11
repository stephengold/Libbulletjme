/*
Contact callbacks that were formerly global.
Now each btDispatcher has its own instance.

header file added by stephengold 2026-07-10

*/
#ifndef BT_CONTACT_CALLBACKS_H
#define BT_CONTACT_CALLBACKS_H

class btCollisionObject;
class btManifoldPoint;
class btPersistentManifold;

typedef bool (*ContactConceivedCallback)(
        btManifoldPoint&,
        btPersistentManifold*,
        const btCollisionObject* pBodyA,
        const btCollisionObject* pBodyB);

typedef void (*ContactEndedCallback)(
        btPersistentManifold* const& manifold);

typedef bool (*ContactProcessedCallback)(
        btManifoldPoint& cp,
        void* body0,
        void* body1);

typedef void (*ContactStartedCallback)(
        btPersistentManifold* const& manifold);

class btContactCallbacks
{
public:
        ContactConceivedCallback m_contactConceivedCallback = 0;
        ContactEndedCallback m_contactEndedCallback = 0;
        ContactProcessedCallback m_contactProcessedCallback = 0;
        ContactStartedCallback m_contactStartedCallback = 0;

        btContactCallbacks() {}
	virtual ~btContactCallbacks() {}
};

#endif  //BT_CONTACT_CALLBACKS_H
